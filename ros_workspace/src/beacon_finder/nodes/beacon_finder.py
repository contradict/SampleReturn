#!/usr/bin/env python
import sys
import copy
import rospy
import rosnode
import math
from math import pi
import Queue
import threading
import cv
import cv2
import numpy
import tf
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3, Quaternion, PoseStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_matrix, quaternion_inverse, quaternion_multiply
from image_geometry import PinholeCameraModel

class BeaconFinder:
    """A class to locate the beacon in an image and publish a vector in the camera's frame from the robot to the beacon"""

    def __init__(self):
        self._beacon_pose_publisher = rospy.Publisher('beacon_pose', PoseWithCovarianceStamped, queue_size=1)
        self._beacon_debug_pose_publisher = rospy.Publisher('beacon_pose_debug', PoseStamped, queue_size=1)
        self._beacon_debug_image = rospy.Publisher('beacon_debug_img', Image, queue_size=1)
        self._cv_bridge = CvBridge()

        self._camera_model = None

        self._detect_sides = rospy.get_param("~detect_sides", False)

        # Get params
        self._num_rows = rospy.get_param("~num_rows", 3)
        self._num_columns = rospy.get_param("~num_columns", 9)
        self._blob_color = rospy.get_param("~blob_color", 0)
        self._blob_color_alt = rospy.get_param("~blob_color_alt", 255)
        self._blob_min_area = rospy.get_param("~blob_min_area", 9)
        self._blob_max_area = rospy.get_param("~blob_max_area", 5000)
        self._blob_side_max_area = rospy.get_param("~blob_side_max_area", 10000)
        self._blob_min_threshold = rospy.get_param("~blob_min_threshold", 20)
        self._blob_max_threshold = rospy.get_param("~blob_max_threshold", 220)
        self._blob_threshold_step = rospy.get_param("~blob_threshold_step", 10)
        self._blob_min_distance_between_blobs = rospy.get_param("~blob_min_distance_between_blobs", 3.0)
        self._blob_repeatability = rospy.get_param("~blob_repeatability", 3L)
        self._do_histogram_equalization = rospy.get_param("~do_histogram_equalization", False)

        self._beacon_mounting_frame = rospy.get_param("~beacon_mounting_frame",
                "platform")
        self._world_fixed_frame = rospy.get_param("~world_fixed_frame", "map")
        self._frontback_covariance = rospy.get_param("~frontback_covariance", [1.0, 1.0, 1.0, pi/16.0, pi/16.0, pi/16.0])

        # beacon side params
        self.maxSizeError = rospy.get_param("~max_size_error", 0.1)
        self.maxDistanceError = rospy.get_param("~max_distance_error", 0.05)
        self.maxHorizontalRadians = rospy.get_param("~max_horizontal_radians", 0.1)
        self._side_covariance = rospy.get_param("~side_covariance", [2.0, 2.0, 2.0, pi/8.0, pi/8.0, pi/8.0])

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

        self._blob_detector_side_params = cv2.SimpleBlobDetector_Params()
        self._blob_detector_side_params.blobColor = self._blob_color
        self._blob_detector_side_params.minArea = self._blob_min_area
        self._blob_detector_side_params.maxArea = self._blob_side_max_area
        self._blob_detector_side_params.minThreshold = self._blob_min_threshold
        self._blob_detector_side_params.maxThreshold = self._blob_max_threshold
        self._blob_detector_side_params.thresholdStep = self._blob_threshold_step
        self._blob_detector_side_params.minDistBetweenBlobs = self._blob_min_distance_between_blobs
        self._blob_detector_side_params.minRepeatability = self._blob_repeatability
        self._blob_detector_side_params.filterByColor = True
        self._blob_detector_side_params.filterByArea = True
        self._blob_detector_side_params.filterByCircularity = False
        self._blob_detector_side_params.filterByInertia = False
        self._blob_detector_side_params.filterByConvexity = True

        self._blob_detector_side_params_alt = cv2.SimpleBlobDetector_Params()
        self._blob_detector_side_params_alt.blobColor = self._blob_color_alt
        self._blob_detector_side_params_alt.minArea = self._blob_min_area
        self._blob_detector_side_params_alt.maxArea = self._blob_side_max_area
        self._blob_detector_side_params_alt.minThreshold = self._blob_min_threshold
        self._blob_detector_side_params_alt.maxThreshold = self._blob_max_threshold
        self._blob_detector_side_params_alt.thresholdStep = self._blob_threshold_step
        self._blob_detector_side_params_alt.minDistBetweenBlobs = self._blob_min_distance_between_blobs
        self._blob_detector_side_params_alt.minRepeatability = self._blob_repeatability
        self._blob_detector_side_params_alt.filterByColor = True
        self._blob_detector_side_params_alt.filterByArea = True
        self._blob_detector_side_params_alt.filterByCircularity = False
        self._blob_detector_side_params_alt.filterByInertia = False
        self._blob_detector_side_params_alt.filterByConvexity = True

        self._blob_detector = cv2.SimpleBlobDetector(self._blob_detector_params)
        self._blob_detector_alt = cv2.SimpleBlobDetector(self._blob_detector_params_alt)
        self._blob_detector_side = cv2.SimpleBlobDetector(self._blob_detector_side_params)
        self._blob_detector_side_alt = cv2.SimpleBlobDetector(self._blob_detector_side_params_alt)

        self._camera_model = None

        self._found_queue = Queue.Queue()

        if self._do_histogram_equalization:
            self._image_output_encoding = '8UC1'
        else:
            self._image_output_encoding = 'rgb8'

        # Construct a matrix of points representing the circles grid facing the camera
        d = rospy.get_param("~vertical_distance_between_circles", 0.3048)
        self._beacon_side_distance_between_circles = rospy.get_param("~side_distance_between_circles", 0.3429)
        self._beacon_thickness = rospy.get_param("~beacon_thickness", 0.394)
        self._beacon_width = rospy.get_param("~beacon_width", 1.51765)
        shifted = rospy.get_param("~is_first_column_shifted_down", False)
        self._circles_grid = self.create_circles_grid(d, shifted)

        self._camera_info_subscriber = rospy.Subscriber('camera_info', CameraInfo,
                self.camera_info_callback, queue_size=1)
        self._image_subscriber = rospy.Subscriber('camera_image', Image,
                self.image_callback, queue_size=1, buff_size=1024*1024*64)
        #tf broadcaster stuff
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Timer(rospy.Duration(0.5), self.broadcast_beacon_tf)

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
        else:
            rospy.loginfo("No circles found in %s.", name)

    def sideLook(self, name, image, detector):
        rospy.loginfo("Looking for %s", name)
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

            # draw blobs, for debugging
            self.draw_debug_keypoints_image(topThree)

            # sort by y coordinate to get them stacked (they should be vertical)
            topThree = sorted(topThree, key=lambda b:b.pt[1])
            avgSize = 0.0
            for b in topThree:
                avgSize += b.size
            avgSize /= 3.0

            for b in topThree:
                if (abs(b.size - avgSize) > avgSize*self.maxSizeError):
                    foundSide = False
                    #rospy.logdebug('rejecting points for size inconsistencies')

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
                #rospy.logdebug('rejecting points for distance errors')

            if distance0to1 < 0.00001 or\
               distance1to2 < 0.00001 or\
               math.asin(abs(horizontal0to1)/distance0to1) > self.maxHorizontalRadians or\
               math.asin(abs(horizontal1to2)/distance1to2) > self.maxHorizontalRadians or\
               numpy.sign(horizontal0to1) != numpy.sign(horizontal1to2):
                foundSide = False
                #rospy.logdebug('rejecting points for horizontal error')

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
                            if (distance < avgDistance*(1.0+self.maxDistanceError)) and\
                               (abs(b.size - avgSize) < avgSize*self.maxSizeError):
                                sameSizeCount += 1

                if sameSizeCount >= 1:
                    #rospy.logdebug('found %d similar sized blobs, which is too many!', sameSizeCount)
                    foundSide = False
                        
            # if we made it here and foundSide is still true, break,
            # we must have found it
            if(foundSide):
                cv2.rectangle(self._debug_image_cv, (int(topThree[0].pt[0]-avgSize), int(topThree[0].pt[1]-avgSize)),
                        (int(topThree[2].pt[0]+avgSize), int(topThree[2].pt[1]+avgSize)),
                        (255,0,0),
                        thickness=-1
                )
                self._found_queue.put((name, topThree, foundSide))

                rospy.loginfo('found %s', name)
                break
            else:
                # if we didn't find it, remove one element from sorted blobs and
                # try again.
                sortedBlobs = sortedBlobs[1:]

    def image_callback(self, image):
        # This function receives an image, attempts to locate the beacon
        # in it, then if successful outputs a beacon pose
        found = False
        centers = numpy.array([])
        image_cv = self._cv_bridge.imgmsg_to_cv2(image)
        if self._camera_model is not None:
            if self._do_histogram_equalization:
                image_cv = cv2.cvtColor(image_cv, cv2.COLOR_RGB2GRAY)
                image_cv = cv2.equalizeHist(image_cv)

            while not self._found_queue.empty():
                self._found_queue.get()

            self._debug_image_cv = copy.deepcopy(image_cv)

            front_thread = threading.Thread( target=self.look,
                    args = ("Front", image_cv, self._blob_detector) )
            front_thread.start()
            back_thread = threading.Thread( target=self.look,
                    args = ("Back", image_cv, self._blob_detector_alt))
            back_thread.start()
            if self._detect_sides:
                left_thread = threading.Thread( target=self.sideLook,
                        args = ("Left", image_cv, self._blob_detector_side))
                left_thread.start()
                right_thread = threading.Thread( target=self.sideLook,
                        args = ("Right", image_cv, self._blob_detector_side_alt))
                right_thread.start()

                left_thread.join()
                right_thread.join()

            front_thread.join()
            back_thread.join()

            pose_matrix = None
            covariance_matrix = None
            pose_matrix_side = None
            covariance_matrix_side = None
            while not self._found_queue.empty():
                name, centers, found = self._found_queue.get()
                if name == "Back":
                    transform =    numpy.r_[
                            [[-1, 0,  0,  0]],
                            [[ 0, 1,  0,  0]],
                            [[ 0, 0, -1,  self._beacon_thickness]],
                            [[ 0, 0,  0,  1]]]
                    pose_matrix, covariance_matrix = self.compute_beacon_pose(centers, transform)
                    self.draw_debug_chessboard_image(centers, found)
                elif name == "Front":
                    transform = numpy.eye(4)
                    pose_matrix, covariance_matrix  = self.compute_beacon_pose(centers, transform)
                    self.draw_debug_chessboard_image(centers, found)
                elif name == "Left":
                    transform =    numpy.r_[
                            [[ 0, 0, -1,  self._beacon_thickness/2.0]],
                            [[ 0, 1,  0,  0]],
                            [[ 1, 0,  0,  self._beacon_width/2.0]],
                            [[ 0, 0,  0,  1]]]
                    pose_matrix_side, covariance_matrix_side = self.compute_beacon_pose_side(centers, image_cv, transform)
                elif name == "Right":
                    transform =    numpy.r_[
                            [[ 0, 0,  1,  -self._beacon_thickness/2.0]],
                            [[ 0, 1,  0,  0]],
                            [[-1, 0,  0,  self._beacon_width/2.0]],
                            [[ 0, 0,  0,  1]]]
                    pose_matrix_side, covariance_matrix_side = self.compute_beacon_pose_side(centers, image_cv, transform)

            if pose_matrix is not None and covariance_matrix is not None:
                self.send_beacon_message(pose_matrix, covariance_matrix, image.header)
            elif pose_matrix_side is not None and covariance_matrix_side is not None:
                self.send_beacon_message(pose_matrix_side, covariance_matrix_side, image.header)

            # Debug image output
            self._beacon_debug_image.publish(self._cv_bridge.cv2_to_imgmsg(self._debug_image_cv, self._image_output_encoding))

        rospy.loginfo("Latency: %f", (rospy.Time.now() - image.header.stamp).to_sec())

    def compute_beacon_pose(self, centers, transform):
        # Compute the position and orientation of the beacon
        rotation_vector, translation_vector, inliers = cv2.solvePnPRansac(
                self._circles_grid,
                centers,
                numpy.asarray(self._camera_model.fullIntrinsicMatrix()),
                numpy.asarray(self._camera_model.distortionCoeffs()))

        rospy.loginfo("solvePnP inliers: %d", len(inliers))
        rospy.loginfo("Translation vector: %s", translation_vector)

        rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
        pose_matrix = numpy.vstack((rotation_matrix,
            numpy.zeros((1,3))))
        pose_matrix = numpy.hstack((pose_matrix, numpy.r_[translation_vector,[[1]]]))
        pose_matrix = numpy.dot( pose_matrix, transform )

        covariance_matrix = [self._frontback_covariance[0]]
        for variance in self._frontback_covariance[1:]:
            covariance_matrix.extend(6*[0]+[variance])

        return pose_matrix, covariance_matrix

    def compute_beacon_pose_side(self, centers, image, transform):
        # Compute the position and orientation of the beacon
        # x right, y down, z away
        if centers is None or self._camera_model is None or len(centers) != 3:
            return None

        image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        ret, image_binary = cv2.threshold(image_gray, 64, 255, cv2.THRESH_BINARY)
        circle_width_height_ratio = 0
        highest_center_point = centers[0].pt
        for center in centers:
            if (center.pt[1] > highest_center_point[1]):
                highest_center_point = center.pt
            center_point = [int(coord) for coord in center.pt]
            flood_color = image_binary[center_point[1],center_point[0]]
            #rospy.logerr("Center point: %f, %f" % (center_point[0], center_point[1]))
            #rospy.logerr("Flood color: %f" % flood_color)
            bounding_rect = cv.FloodFill(cv.fromarray(image_binary), (center_point[0], center_point[1]), float(flood_color))
            #rospy.logerr("Bounding rect width: %f, height: %f" % (float(bounding_rect[2][2]), float(bounding_rect[2][3])))
            circle_width_height_ratio = circle_width_height_ratio + float(bounding_rect[2][2])/float(bounding_rect[2][3])
            #rospy.logerr("Circle width height ratio: %f" % (circle_width_height_ratio))
        circle_width_height_ratio = circle_width_height_ratio / len(centers)
        
        very_approx_angle = 0.0
        if circle_width_height_ratio > 1.0:
            very_approx_angle = 0.0
        else:
            very_approx_angle = math.acos(circle_width_height_ratio)

        # Compute rotation and translation vectors such that they emulate
        # the output of solvePnP().  This has many hard-coded assumptions about
        # the number of circles on the beacon's sides
        average_circle_distance = math.sqrt((centers[0].pt[0]-centers[1].pt[0])**2 + (centers[0].pt[1]-centers[1].pt[1])**2)
        average_circle_distance = average_circle_distance + math.sqrt((centers[1].pt[0]-centers[2].pt[0])**2 + (centers[1].pt[1]-centers[2].pt[1])**2)
        average_circle_distance = average_circle_distance / 2.0
        beacon_distance = (self._camera_model.fy()/average_circle_distance)*self._beacon_side_distance_between_circles
        beacon_distance_z = math.cos(math.atan((highest_center_point[0] - self._camera_model.cx())/self._camera_model.fx()))*beacon_distance
        x_meters = (highest_center_point[0] - self._camera_model.cx())/self._camera_model.fx()*beacon_distance_z
        y_meters = (highest_center_point[1] - self._camera_model.cy())/self._camera_model.fy()*beacon_distance_z
        rotation_vector = numpy.r_[0, very_approx_angle, 0]
        translation_vector = numpy.r_[[[x_meters]], [[y_meters]], [[beacon_distance]]]
        rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
        pose_matrix = numpy.vstack((rotation_matrix, numpy.zeros((1,3))))
        pose_matrix = numpy.hstack((pose_matrix, numpy.r_[translation_vector,[[1]]]))
        pose_matrix = numpy.dot(pose_matrix, transform)

        #rospy.logerr("Beacon dist: %f, beacon dist z: %f" % (beacon_distance, beacon_distance_z))
        #rospy.logerr("X dist: %f, Y dist: %f" % (x_meters, y_meters))
        #rospy.logerr("Rotation angle: %f" % (very_approx_angle*180.0/pi))
        #rospy.logerr("Circle width/height ratio: %f" % (circle_width_height_ratio))

        covariance_matrix = [self._side_covariance[0]]
        for variance in self._side_covariance[1:]:
            covariance_matrix.extend(6*[0]+[variance])
        return pose_matrix, covariance_matrix
            
        

    def send_beacon_message(self, pose_matrix, covariance_matrix, header):
        quat = quaternion_from_matrix(pose_matrix)

        beacon_pose = PoseWithCovarianceStamped()
        beacon_pose.pose.pose.position.x = pose_matrix[0, -1]
        beacon_pose.pose.pose.position.y = pose_matrix[1, -1]
        beacon_pose.pose.pose.position.z = pose_matrix[2, -1]
        beacon_pose.pose.pose.orientation.x = quat[0]
        beacon_pose.pose.pose.orientation.y = quat[1]
        beacon_pose.pose.pose.orientation.z = quat[2]
        beacon_pose.pose.pose.orientation.w = quat[3]
        beacon_pose.header = header
        beacon_pose.pose.covariance = covariance_matrix
        self._beacon_pose_publisher.publish(beacon_pose)

        debug_pose = PoseStamped()
        debug_pose.pose = beacon_pose.pose.pose
        debug_pose.header = header
        self._beacon_debug_pose_publisher.publish(debug_pose)

    def draw_debug_keypoints_image(self, keypoints):
        if self._beacon_debug_image.get_num_connections() > 0:
            self._debug_image_cv = cv2.drawKeypoints(self._debug_image_cv, keypoints, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    def draw_debug_chessboard_image(self, centers, found):
        if self._beacon_debug_image.get_num_connections() > 0:
            cv2.drawChessboardCorners(self._debug_image_cv,
                    (self._num_rows, self._num_columns),
                    centers,
                    found)

    def camera_info_callback(self, camera_info):
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)
        self._camera_model = camera_model

    def broadcast_beacon_tf(self, evt):
        now = rospy.Time.now()
        #beacon_trans = (0.0, -1.3, -1.0)
        beacon_trans = (-0.780, 0.0, 0.712)
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

