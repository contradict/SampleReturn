#!/usr/bin/env python
import sys
import rospy
import rosnode
import math
import Queue
import threading
import cv2
import numpy
import tf
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped
from tf.transformations import quaternion_from_matrix, quaternion_inverse, quaternion_multiply
from image_geometry import PinholeCameraModel

class BeaconFinder:
    """A class to locate the beacon in an image and publish a vector in the camera's frame from the robot to the beacon"""

    def __init__(self):
        self._image_subscriber = rospy.Subscriber('camera_image', Image,
                self.image_callback, queue_size=1, buff_size=1024*1024*64)
        self._camera_info_subscriber = rospy.Subscriber('camera_info', CameraInfo,
                self.camera_info_callback, queue_size=1)
        self._beacon_pose_publisher = rospy.Publisher('beacon_pose', PoseStamped)
        self._beacon_debug_image = rospy.Publisher('beacon_debug_img', Image)
        self._cv_bridge = CvBridge()

        #tf broadcaster stuff
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Timer(rospy.Duration(0.5), self.broadcast_beacon_tf) 

        # Get params
        self._num_rows = rospy.get_param("~num_rows", 3)
        self._num_columns = rospy.get_param("~num_columns", 9)
        self._blob_color = rospy.get_param("~blob_color", 0)
        self._blob_color_alt = rospy.get_param("~blob_color_alt", 255)
        self._blob_min_area = rospy.get_param("~blob_min_area", 9)
        self._blob_max_area = rospy.get_param("~blob_max_area", 5000)
        self._blob_min_threshold = rospy.get_param("~blob_min_threshold", 20)
        self._blob_max_threshold = rospy.get_param("~blob_max_threshold", 220)
        self._blob_threshold_step = rospy.get_param("~blob_threshold_step", 10)
        self._blob_min_distance_between_blobs = rospy.get_param("~blob_min_distance_between_blobs", 3.0)
        self._blob_repeatability = rospy.get_param("~blob_repeatability", 3L)
        self._do_histogram_equalization = rospy.get_param("~do_histogram_equalization", False)

        self._beacon_mounting_frame = rospy.get_param("~beacon_mounting_frame",
                "platform")
        self._world_fixed_frame = rospy.get_param("~world_fixed_frame", "map")

        # beacon side params
        self.maxSizeError = rospy.get_param("~max_size_error", 0.1)
        self.maxDistanceError = rospy.get_param("~max_distance_error", 0.05)
        self.maxHorizontalError = rospy.get_param("~max_horizontal_error", 0.1)

        # Initialize member variables
        self._blob_detector_params = cv2.SimpleBlobDetector_Params()
        self._blob_detector_params.blobColor = self._blob_color
        self._blob_detector_params.minArea = self._blob_min_area
        self._blob_detector_params.maxArea = self._blob_max_area
        self._blob_detector_params.minThreshold = self._blob_min_threshold
        self._blob_detector_params.maxThreshold = self._blob_max_threshold
        self._blob_detector_params.thresholdStep = self._blob_threshold_step
        self._blob_detector_params.minDistBetweenBlobs = self._blob_min_distance_between_blobs
        self._blob_detector_params.minRepeatability = self._blob_repeatability
        self._blob_detector_params.filterByColor = True
        self._blob_detector_params.filterByArea = True
        self._blob_detector_params.filterByCircularity = False
        self._blob_detector_params.filterByInertia = False
        self._blob_detector_params.filterByConvexity = True

        self._blob_detector_params_alt = cv2.SimpleBlobDetector_Params()
        self._blob_detector_params_alt.blobColor = self._blob_color_alt
        self._blob_detector_params_alt.minArea = self._blob_min_area
        self._blob_detector_params_alt.maxArea = self._blob_max_area
        self._blob_detector_params_alt.minThreshold = self._blob_min_threshold
        self._blob_detector_params_alt.maxThreshold = self._blob_max_threshold
        self._blob_detector_params_alt.thresholdStep = self._blob_threshold_step
        self._blob_detector_params_alt.minDistBetweenBlobs = self._blob_min_distance_between_blobs
        self._blob_detector_params_alt.minRepeatability = self._blob_repeatability
        self._blob_detector_params_alt.filterByColor = True
        self._blob_detector_params_alt.filterByArea = True
        self._blob_detector_params_alt.filterByCircularity = False
        self._blob_detector_params_alt.filterByInertia = False
        self._blob_detector_params_alt.filterByConvexity = True

        self._blob_detector = cv2.SimpleBlobDetector(self._blob_detector_params)
        self._blob_detector_alt = cv2.SimpleBlobDetector(self._blob_detector_params_alt)
        self._camera_model = None

        self._found_queue = Queue.Queue()

        if self._do_histogram_equalization:
            self._image_output_encoding = '8UC1'
        else:
            self._image_output_encoding = 'rgb8'

        # Construct a matrix of points representing the circles grid facing the camera
        d = rospy.get_param("~vertical_distance_between_circles", 0.3048)
        self._beacon_thickness = rospy.get_param("~beacon_thickness", 0.394)
        shifted = rospy.get_param("~is_first_column_shifted_down", False)
        self._circles_grid = self.create_circles_grid(d, shifted)

    def create_circles_grid(self, d, shifted):
        if shifted:
            offset = -d/2.0
        else:
            offset = 0.0
        circles_grid = numpy.array([0,0,0], dtype="float32") # Create a 'dummy' row
        for column in range(-self._num_columns/2+1, self._num_columns/2+1):
            for row in range(0, -self._num_rows, -1):
                circles_grid = numpy.vstack((circles_grid, numpy.array([column*d/2.0,row*d+offset,0.0])))
            if offset == 0.0:
                offset = -d/2.0
            else:
                offset = 0.0
        circles_grid = numpy.delete(circles_grid, 0, 0) # Delete the 'dummy' row
        circles_grid = numpy.array(circles_grid, dtype="float32") # Ensure float32 data type, which shouldn't be necessary at this point, but it is...
        return circles_grid

    def look(self, name, image, detector):
        rospy.loginfo("Looking for %s", name)
        found, centers = cv2.findCirclesGrid(image,
                (self._num_rows, self._num_columns),
                flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
                blobDetector=detector)
        if found:
            rospy.loginfo("Found %s", name)
            self._found_queue.put((name, centers, found))

    def sideLook(self, name, image, detector):
        rospy.logerr("Looking for %s", name)
        foundSide = True
        blobs = detector.detect(image)

        # sort by size, largest first
        sortedBlobs = sorted(blobs, key=lambda b:b.size, reverse=True)

        # sometimes we see a huge blob that's not related, so loop until we're
        # out of blobs to test
        while(len(sortedBlobs) >= 3):
            # reset the flag so we can look at the value at the end of the loop
            foundSide = True

            topThree = sortedBlobs[:3]
            # sort by y coordinate to get them stacked (they should be vertical)
            topThree = sorted(topThree, key=lambda b:b.pt[1])
            avgSize = 0.0
            for b in topThree:
                avgSize += b.size
            avgSize /= 3.0

            for i in range(len(topThree)):
                b = topThree[i]
                if (b.size < avgSize * (1.0 - self.maxSizeError)) or\
                        (b.size > avgSize * (1.0 + self.maxSizeError)):
                    foundSide = False
                    rospy.logerr('rejecting points for size inconsistencies')

            horizontal0to1 = topThree[0].pt[0]-topThree[1].pt[0]
            horizontal1to2 = topThree[1].pt[0]-topThree[2].pt[0]

            distance0to1 = numpy.sqrt((topThree[0].pt[0]-topThree[1].pt[0])**2 + \
                    (topThree[0].pt[1]-topThree[1].pt[1])**2)
            distance1to2 = numpy.sqrt((topThree[1].pt[0]-topThree[2].pt[0])**2 + \
                    (topThree[1].pt[1]-topThree[2].pt[1])**2)

            avgDistance = (distance0to1 + distance1to2)/2.0

            if abs(distance0to1 - distance1to2) > \
                    min(distance0to1, distance1to2) * self.maxDistanceError:
                foundSide = False
                rospy.logerr('rejecting points for distance errors')

            if abs(horizontal0to1 - horizontal1to2) > \
                    abs(min(horizontal0to1, horizontal1to2)) * self.maxHorizontalError:
                foundSide = False
                rospy.logerr('rejecting points for horizontal error')

            # sometimes we see the front or back and think it is a side.
            # deal with this by not allowing there to be too many similar sized blobs
            # in the image. hopefully this lowers detection rates.
            if foundSide:
                sameSizeCount = 0
                for b in blobs:
                    if not b in topThree:
                        for c in topThree:
                            distance = numpy.sqrt((c.pt[0]-b.pt[0])**2 + \
                                    (c.pt[1]-b.pt[1])**2)
                            if (distance > (1.0-self.maxDistanceError)*avgDistance) and\
                                    (distance < (1.0+self.maxDistanceError)*avgDistance):
                                sameSizeCount += 1

                if sameSizeCount >= 2:
                    rospy.logerr('found %d similar sized blobs, which is too many!', sameSizeCount)
                    foundSide = False
                        
            # if we made it here and foundSide is still true, break,
            # we must have found it
            if(foundSide):
                cv2.rectangle(image, (int(topThree[0].pt[0]-avgSize), int(topThree[0].pt[1]-avgSize)),
                        (int(topThree[2].pt[0]+avgSize), int(topThree[2].pt[1]+avgSize)),
                        (255,0,0),
                        thickness=-1
                )
                self._beacon_debug_image.publish(self._cv_bridge.cv2_to_imgmsg(image, self._image_output_encoding))
                rospy.logerr('found %s', name)
                break
            else:
                # if we didn't find it, remove one element from sorted blobs and
                # try again.
                sortedBlobs = sortedBlobs[1:]

    def image_callback(self, image):
        # This function receives an image, attempts to locate the beacon
        # in it, then if successful outputs a vector towards it in the
        # camera's frame
        found = False
        centers = numpy.array([])
        image_cv = self._cv_bridge.imgmsg_to_cv2(image)
        if self._camera_model is not None:
            if self._do_histogram_equalization:
                image_cv = cv2.cvtColor(image_cv, cv2.COLOR_RGB2GRAY)
                image_cv = cv2.equalizeHist(image_cv)

            while not self._found_queue.empty():
                self._found_queue.get()

            front_thread = threading.Thread( target=self.look,
                    args = ("Front", image_cv, self._blob_detector) )
            front_thread.start()
            back_thread = threading.Thread( target=self.look,
                    args = ("Back", image_cv, self._blob_detector_alt))
            back_thread.start()
            left_thread = threading.Thread( target=self.sideLook,
                    args = ("Left", image_cv, self._blob_detector))
            left_thread.start()
            right_thread = threading.Thread( target=self.sideLook,
                    args = ("Right", image_cv, self._blob_detector_alt))
            right_thread.start()

            left_thread.join()
            right_thread.join()
            front_thread.join()
            back_thread.join()

            while not self._found_queue.empty():
                name, centers, found = self._found_queue.get()
                if name == "Back":
                    transform =    numpy.r_[[[-1, 0,  0,  0]],
                            [[ 0, 1,  0,  0]],
                            [[ 0, 0, -1, -self._beacon_thickness]],
                            [[ 0, 0,  0,  1]]]
                else:
                    transform = numpy.eye(4)
                pose_matrix = self.compute_beacon_pose(centers, transform)
                self.send_beacon_message(pose_matrix, image.header)

        rospy.loginfo("Latency: %f", (rospy.Time.now() - image.header.stamp).to_sec())
        self.send_debug_image( image_cv, centers, found)

    def compute_beacon_pose(self, centers, transform):
        # Compute the position and orientation of the beacon
        rotation_vector, translation_vector, inliers = cv2.solvePnPRansac(
                self._circles_grid,
                centers,
                numpy.asarray(self._camera_model.fullIntrinsicMatrix()),
                numpy.asarray(self._camera_model.distortionCoeffs()))

        rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
        pose_matrix = numpy.vstack((rotation_matrix,
            numpy.zeros((1,3))))
        pose_matrix = numpy.hstack((pose_matrix, numpy.r_[translation_vector,[[1]]]))
        pose_matrix = numpy.dot( pose_matrix, transform )

        return pose_matrix

    def send_beacon_message(self, pose_matrix, header):
        quat = quaternion_from_matrix(pose_matrix)

        beacon_pose = PoseStamped()
        beacon_pose.pose.position.x = pose_matrix[0, -1]
        beacon_pose.pose.position.y = pose_matrix[1, -1]
        beacon_pose.pose.position.z = pose_matrix[2, -1]
        beacon_pose.pose.orientation.x = quat[0]
        beacon_pose.pose.orientation.y = quat[1]
        beacon_pose.pose.orientation.z = quat[2]
        beacon_pose.pose.orientation.w = quat[3]
        beacon_pose.header = header
        self._beacon_pose_publisher.publish(beacon_pose)

    def send_debug_image(self, image, centers, found):
        if self._beacon_debug_image.get_num_connections() > 0:
            cv2.drawChessboardCorners(image,
                    (self._num_rows, self._num_columns),
                    centers,
                    found)
            self._beacon_debug_image.publish(self._cv_bridge.cv2_to_imgmsg(image, self._image_output_encoding))

    def camera_info_callback(self, camera_info):
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)
        self._camera_model = camera_model

    def broadcast_beacon_tf(self, evt):
        now = rospy.Time.now()
        #beacon_trans = (0.0, -1.3, -1.0)
        beacon_trans = (-1.22, 0.0, 1.48)
        beacon_rot = tf.transformations.quaternion_from_euler(
                -math.pi/2,
                -math.pi/2,
                0.0,
                'rxyz')
        self.tf_broadcaster.sendTransform(beacon_trans,
                beacon_rot,
                now,
                '/beacon',
                '/platform',
                )

        if self._beacon_mounting_frame != self._world_fixed_frame:
            zero_translation = (0,0,0)
            zero_rotation = (0,0,0,1)
            self.tf_broadcaster.sendTransform(zero_translation,
                    zero_rotation,
                    now,
                    self._beacon_mounting_frame,
                    self._world_fixed_frame)

if __name__ == '__main__':
    rospy.init_node('beacon_finder')
    beacon_finder = BeaconFinder()
    rospy.spin()

