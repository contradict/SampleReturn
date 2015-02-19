#include <stdint.h>
#include <stdio.h>
#include <gpuimageproc/connectedtopics.h>

int main(int argc, char **argv)
{
    struct gpuimageproc::ConnectedTopics ct1;

    printf("None: %d\n", ct1.level());
    ct1.DebayerMono = 1;
    printf("DebayerMono: %d\n", ct1.level());
    ct1.DebayerMono = 0;
    ct1.Pointcloud = 1;
    printf("Pointcloud: %d\n", ct1.level());
}
