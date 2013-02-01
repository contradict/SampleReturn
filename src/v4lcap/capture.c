#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <sys/select.h>
#include <assert.h>
#include <png.h>

struct buffer {
    void * start;
    size_t length;
};


int main(int argc, char * argv[])
{
    char *dev_names[] = {"/dev/video0", "/dev/video1"};
    int fds[2];
    struct buffer * buffers[2];
    int buff_count[2];
    struct v4l2_capability cap;
    struct v4l2_format format;
    struct v4l2_requestbuffers req;
    struct v4l2_buffer buf;
    int i, j, idx, n_buffers;
    struct v4l2_streamparm streamp;
    struct v4l2_frmivalenum frmiv;
    enum v4l2_buf_type type;
    int count=100, fc[2] = {0,0};
    int wfd;
    char wfname[255];
    struct timeval timeout;
    int r;
    int maxfd;
    fd_set fdset;

    for(i=0;i<2;i++) {
        fds[i] = open(dev_names[i], O_RDWR | O_NONBLOCK, 0);
        if(fds[i] == -1) {
            perror("Could not open device");
            exit(1);
        }
        memset( &cap, 0, sizeof(cap) );
        if( -1 == ioctl(fds[i], VIDIOC_QUERYCAP, &cap)) {
            perror("Could not query capabilities");
            exit(1);
        }
        if( !(cap.capabilities & V4L2_CAP_STREAMING) ) {
            fprintf(stderr, "%s does not support streaming IO\n", dev_names[i]);
            exit(1);
        }

        memset( &frmiv, 0, sizeof(frmiv));
        frmiv.pixel_format = V4L2_PIX_FMT_YUYV;
        frmiv.width = 640;
        frmiv.height = 480;
        fprintf(stderr, "Frame intervals: ");
        for(idx=0;;idx++){
            frmiv.index = idx;
            if( -1 == ioctl(fds[i], VIDIOC_ENUM_FRAMEINTERVALS, &frmiv) ) {
                if((frmiv.type == V4L2_FRMIVAL_TYPE_DISCRETE) &&
                        (EINVAL == errno)) {
                    fprintf(stderr, "\n");
                    break;
                }
                perror("VIDIOC_ENUM_FRAMEINTERVALS");
                break;
            }
            switch(frmiv.type) {
                case V4L2_FRMIVAL_TYPE_DISCRETE:
                    fprintf(stderr, "%d/%d, ",
                            frmiv.discrete.numerator,
                            frmiv.discrete.denominator);
                    break;
                case V4L2_FRMIVAL_TYPE_STEPWISE:
                    fprintf(stderr, "%d/%d - %d/%d by %d/%d\n",
                            frmiv.stepwise.min.numerator,frmiv.stepwise.min.denominator,
                            frmiv.stepwise.max.numerator,frmiv.stepwise.max.denominator,
                            frmiv.stepwise.step.numerator,frmiv.stepwise.step.denominator);
                    break;
                case V4L2_FRMIVAL_TYPE_CONTINUOUS:
                    fprintf(stderr, "%d/%d - %d/%d\n",
                            frmiv.stepwise.min.numerator,frmiv.stepwise.min.denominator,
                            frmiv.stepwise.max.numerator,frmiv.stepwise.max.denominator);
                    break;
                default:
                    fprintf(stderr, "Unexpected type %d\n", frmiv.type);
                    break;
            }
            if(frmiv.type != V4L2_FRMIVAL_TYPE_DISCRETE) break;
        }

        memset( &format, 0, sizeof(format) );
        format.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width       = 640;
        format.fmt.pix.height      = 480;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        format.fmt.pix.field       = V4L2_FIELD_NONE;
        if( -1 == ioctl(fds[i], VIDIOC_S_FMT, &format) ) {
            perror("Could not set video format");
            exit(1);
        }

        memset( &streamp, 0, sizeof(streamp) );
        streamp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        streamp.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
        streamp.parm.capture.timeperframe.numerator=1;
        streamp.parm.capture.timeperframe.denominator=15;
        if( -1 == ioctl(fds[i], VIDIOC_S_PARM, &streamp) ) {
            perror("VIDIOC_S_PARM");
            exit(1);
        }
        fprintf(stderr, "set framerate to %d/%d FPS\n",
                streamp.parm.capture.timeperframe.denominator,
                streamp.parm.capture.timeperframe.numerator
               );


        memset( &req, 0, sizeof(req));

        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if( -1 == ioctl(fds[i], VIDIOC_REQBUFS, &req) ) {
            if( EINVAL == errno ) {
                fprintf(stderr, "mmap not supported on %s", dev_names[i]);
                exit(1);
            } else {
                perror("could not request mmap buffer");
                exit(1);
            }
        }

        if(req.count < 2 ) {
            fprintf(stderr, "Insufficient memory for mmap on %s", dev_names[i]);
            exit(1);
        }

        buffers[i] = calloc(req.count, sizeof(struct buffer));
        buff_count[i] = req.count;

        for(n_buffers=0;n_buffers<req.count;++n_buffers) {

            memset( &buf, 0, sizeof(buf));

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = n_buffers;

            if( -1 == ioctl(fds[i], VIDIOC_QUERYBUF, &buf) ) {
                perror("VIDIOC_QUERYBUF");
                exit(1);
            }
            buffers[i][n_buffers].length = buf.length;
            buffers[i][n_buffers].start = 
                mmap(NULL,
                     buf.length,
                     PROT_READ | PROT_WRITE,
                     MAP_SHARED,
                     fds[i],
                     buf.m.offset);
            if( MAP_FAILED == buffers[i][n_buffers].start ) {
                perror("mmap");
                exit(1);
            }
        }
    }

    for(i=0;i<2;i++) {
        for(j = 0; j < buff_count[i]; j++ ) {
            memset( &buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = j;
            if( -1 == ioctl(fds[i], VIDIOC_QBUF, &buf) ) {
                perror("Could not enqueue buffer");
                exit(1);
            }
        }
    }

    for(i=0;i<2;i++) {
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fprintf(stderr, "start capture for %s\n", dev_names[i]);
        if( -1 == ioctl(fds[i], VIDIOC_STREAMON, &type) ) {
            perror("Could not start capture");
            exit(1);
        }
    }

    maxfd = fds[0];
    if(fds[1]>maxfd) maxfd = fds[1];

    while( count -- > 0 ) {
        for(;;) {

            FD_ZERO( &fdset );
            FD_SET( fds[0], &fdset);
            FD_SET( fds[1], &fdset);

            timeout.tv_sec = 2;
            timeout.tv_usec = 0;

            r = select( maxfd + 1, &fdset, NULL, NULL, &timeout);

            if( -1 == r ) {
                if( EINTR == errno )
                    continue;
                perror("select");
                exit(1);
            }

            if( 0 == r ) {
                fprintf(stderr, "timeout\n");
                exit(1);
            }

            for(i=0;i<2;i++) {
                if(FD_ISSET(fds[i], &fdset)) {
                    memset( &buf, 0, sizeof(buf) );
                    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                    buf.memory = V4L2_MEMORY_MMAP;
                    for(;;)
                        if( -1 == ioctl( fds[i], VIDIOC_DQBUF, &buf ) ) {
                            switch( errno ) {
                                case EAGAIN:
                                    continue;
                                default:
                                    perror("VIDIOC_DQBUF");
                                    exit(1);
                            }
                        } else break;
                    assert( buf.index < buff_count[i] );
                    printf("%d %f\n", i, buf.timestamp.tv_sec + buf.timestamp.tv_usec/1e6);
                    snprintf(wfname, 255, "camera_%02d_frame_%03d.raw", i,
                            fc[i]);
                    wfd = open(wfname, O_WRONLY|O_CREAT|O_TRUNC, 00666);
                    if( -1==wfd) perror("open");
                    else {
                        write(wfd, &buf.timestamp, sizeof(struct timeval));
                        write(wfd, buffers[i][buf.index].start,
                                buffers[i][buf.index].length);
                        close(wfd);
                    }
                    fc[i] += 1;
                    if( -1 == ioctl(fds[i], VIDIOC_QBUF, &buf) ) {
                        perror("read enqueue");
                        exit(1);
                    }
                }
            }
            break;
        }
    }
    for(i=0;i<2;i++) {
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if( -1 == ioctl(fds[i], VIDIOC_STREAMOFF, &type)) {
            perror("VIDIOC_STREAMOFF");
            exit(1);
        }
        for(j=0;j<buff_count[i];j++)
            if( -1 == munmap(buffers[i][j].start, buffers[i][j].length) ) {
                perror("munmap");
                exit(1);
            }
        close(fds[i]);
    }

    return 0;
}

