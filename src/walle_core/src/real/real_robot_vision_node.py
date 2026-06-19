#!/usr/bin/env python3
# real_robot_vision_node.py
# 真机 Kinect2 视觉检测，与 stimulation/real_vision_node.py（仿真）独立。

import rospy
import cv2
import numpy as np
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PointStamped
from cv_bridge import CvBridge
import tf2_geometry_msgs  # noqa: F401 - required for PointStamped transform


class RealVisionNode:
    def __init__(self):
        rospy.init_node('real_vision_node')

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.depth_image = None
        self.table_height = rospy.get_param('~table_height', 0.75)
        self.show_opencv_window = rospy.get_param('~show_opencv_window', False)
        self.camera_frame = rospy.get_param('~camera_frame', 'kinect2_rgb_optical_frame')
        self.target_frame = rospy.get_param('~target_frame', 'map')
        self.publish_map_detection = rospy.get_param('~publish_map_detection', True)
        self.rgb_topic = rospy.get_param('~rgb_topic', '/kinect2/hd/image_color_rect')
        self.depth_topic = rospy.get_param('~depth_topic', '/kinect2/sd/image_depth_rect')

        self.hd_fx = self.hd_fy = 525.0
        self.hd_cx = 960.0
        self.hd_cy = 540.0
        self.hd_width = 1920
        self.hd_height = 1080

        self.sd_fx = self.sd_fy = 525.0
        self.sd_cx = 256.0
        self.sd_cy = 212.0
        self.sd_width = 512
        self.sd_height = 424

        self.sub_rgb = rospy.Subscriber(
            self.rgb_topic, Image, self.rgb_cb, queue_size=1)
        self.sub_depth = rospy.Subscriber(
            self.depth_topic, Image, self.depth_cb, queue_size=1)
        self.sub_hd_info = rospy.Subscriber(
            '/kinect2/hd/camera_info', CameraInfo, self.hd_info_cb, queue_size=1)
        self.sub_sd_info = rospy.Subscriber(
            '/kinect2/sd/camera_info', CameraInfo, self.sd_info_cb, queue_size=1)

        self.detection_pub = rospy.Publisher('/vision/box_detected', Pose, queue_size=10)
        # Camera-frame detection for grasp alignment:
        #   position.x: camera optical x, meters, right positive
        #   position.y: camera optical y, meters, down positive
        #   position.z: depth to this detected box, meters
        #   orientation.x/y: normalized pixel error from image center
        #   orientation.z: contour area in pixels
        #   orientation.w: color code
        self.camera_detection_pub = rospy.Publisher(
            '/vision/box_camera', Pose, queue_size=10)
        self.debug_image_pub = rospy.Publisher('/vision/debug_image', Image, queue_size=1)

        self.detection_count = 0
        self.frame_count = 0

        rospy.loginfo('真机视觉节点已启动 (Kinect2 bridge)')
        rospy.loginfo('  彩色: %s', self.rgb_topic)
        rospy.loginfo('  深度: %s', self.depth_topic)
        rospy.loginfo('  调试画面: /vision/debug_image')
        rospy.loginfo('  OpenCV窗口: %s', self.show_opencv_window)

    def hd_info_cb(self, msg):
        self.hd_fx = msg.K[0]
        self.hd_fy = msg.K[4]
        self.hd_cx = msg.K[2]
        self.hd_cy = msg.K[5]
        self.hd_width = msg.width
        self.hd_height = msg.height

    def sd_info_cb(self, msg):
        self.sd_fx = msg.K[0]
        self.sd_fy = msg.K[4]
        self.sd_cx = msg.K[2]
        self.sd_cy = msg.K[5]
        self.sd_width = msg.width
        self.sd_height = msg.height

    def depth_cb(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.sd_height, self.sd_width = self.depth_image.shape[:2]
        except Exception as e:
            rospy.logwarn('深度图像转换失败: %s', e)
            self.depth_image = None

    def hd_to_sd_coordinates(self, hd_x, hd_y):
        if self.hd_width <= 0 or self.hd_height <= 0:
            return 0, 0
        sd_x = int(hd_x * self.sd_width / self.hd_width)
        sd_y = int(hd_y * self.sd_height / self.hd_height)
        sd_x = max(0, min(sd_x, self.sd_width - 1))
        sd_y = max(0, min(sd_y, self.sd_height - 1))
        return sd_x, sd_y

    def depth_to_meters(self, depth_value):
        if np.isnan(depth_value):
            return None
        if depth_value > 20:
            return float(depth_value) / 1000.0
        return float(depth_value)

    def pixel_to_map(self, sd_x, sd_y, depth_meters):
        if depth_meters is None or depth_meters <= 0.1 or depth_meters > 3.0:
            return None

        # 相机光学坐标系: x右, y下, z前
        cam_x = depth_meters * (sd_x - self.sd_cx) / self.sd_fx
        cam_y = depth_meters * (sd_y - self.sd_cy) / self.sd_fy
        cam_z = depth_meters

        pt_cam = PointStamped()
        pt_cam.header.frame_id = self.camera_frame
        pt_cam.header.stamp = rospy.Time(0)
        pt_cam.point.x = cam_x
        pt_cam.point.y = cam_y
        pt_cam.point.z = cam_z

        try:
            pt_map = self.tf_buffer.transform(
                pt_cam, self.target_frame, rospy.Duration(1.0))
            return (pt_map.point.x, pt_map.point.y, self.table_height)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, 'TF变换失败 (%s -> %s): %s',
                                 self.camera_frame, self.target_frame, e)
            return None

    def rgb_cb(self, msg):
        # 关闭过程中不再处理/发布，避免 "publish() to a closed topic" 报错
        if rospy.is_shutdown():
            return
        self.frame_count += 1

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr('彩色图像转换失败: %s', e)
            return

        self.hd_height, self.hd_width = cv_image.shape[:2]
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        color_ranges = {
            'red': [
                ([0, 120, 70], [10, 255, 255]),
                ([170, 120, 70], [180, 255, 255])
            ],
            'green': [([40, 50, 50], [80, 255, 255])],
            'yellow': [([20, 100, 100], [30, 255, 255])],
            'blue': [([100, 50, 50], [130, 255, 255])]
        }

        color_code_map = {
            'red': 1.0,
            'green': 2.0,
            'yellow': 3.0,
            'blue': 4.0,
            'brown': 5.0
        }

        for color_name, ranges in color_ranges.items():
            mask = None
            for (lower, upper) in ranges:
                lower_np = np.array(lower, dtype=np.uint8)
                upper_np = np.array(upper, dtype=np.uint8)
                if mask is None:
                    mask = cv2.inRange(hsv_image, lower_np, upper_np)
                else:
                    mask = cv2.bitwise_or(mask, cv2.inRange(hsv_image, lower_np, upper_np))

            if mask is None:
                continue

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            candidates = []
            for contour in contours:
                if cv2.contourArea(contour) < 500:
                    continue

                x, y, w, h = cv2.boundingRect(contour)
                hd_cx = x + w // 2
                hd_cy = y + h // 2
                sd_cx, sd_cy = self.hd_to_sd_coordinates(hd_cx, hd_cy)

                depth_meters = None
                if self.depth_image is not None:
                    if (0 <= sd_cy < self.depth_image.shape[0] and
                            0 <= sd_cx < self.depth_image.shape[1]):
                        depth_meters = self.depth_to_meters(
                            self.depth_image[sd_cy, sd_cx])

                if depth_meters is None or depth_meters <= 0.1:
                    continue

                cam_x = depth_meters * (sd_cx - self.sd_cx) / self.sd_fx
                cam_y = depth_meters * (sd_cy - self.sd_cy) / self.sd_fy
                candidates.append({
                    'x': x, 'y': y, 'w': w, 'h': h,
                    'hd_cx': hd_cx, 'hd_cy': hd_cy,
                    'sd_cx': sd_cx, 'sd_cy': sd_cy,
                    'depth': depth_meters,
                    'cam_x': cam_x, 'cam_y': cam_y,
                    'area': float(cv2.contourArea(contour)),
                })

            if not candidates:
                continue

            # 同色多个目标时只认最近的（深度最小），避免抓到远处箱子
            nearest = min(candidates, key=lambda d: d['depth'])

            for det in candidates:
                if det is nearest:
                    continue
                x, y, w, h = det['x'], det['y'], det['w'], det['h']
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (128, 128, 128), 1)
                cv2.putText(cv_image, '%s %.2fm(远)' % (color_name, det['depth']),
                            (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)

            x, y, w, h = nearest['x'], nearest['y'], nearest['w'], nearest['h']
            hd_cx, hd_cy = nearest['hd_cx'], nearest['hd_cy']
            sd_cx, sd_cy = nearest['sd_cx'], nearest['sd_cy']
            depth_meters = nearest['depth']
            cam_x = nearest['cam_x']

            camera_msg = Pose()
            camera_msg.position.x = cam_x
            camera_msg.position.y = nearest['cam_y']
            camera_msg.position.z = depth_meters
            camera_msg.orientation.x = (
                float(hd_cx) - self.hd_width * 0.5) / (self.hd_width * 0.5)
            camera_msg.orientation.y = (
                float(hd_cy) - self.hd_height * 0.5) / (self.hd_height * 0.5)
            camera_msg.orientation.z = nearest['area']
            camera_msg.orientation.w = color_code_map.get(color_name, 0.0)
            try:
                self.camera_detection_pub.publish(camera_msg)
            except rospy.ROSException:
                return

            world_pos = None
            if self.publish_map_detection:
                world_pos = self.pixel_to_map(sd_cx, sd_cy, depth_meters)

            if world_pos is not None:
                detection_msg = Pose()
                detection_msg.position.x = world_pos[0]
                detection_msg.position.y = world_pos[1]
                detection_msg.position.z = world_pos[2]
                detection_msg.orientation.w = color_code_map.get(color_name, 0.0)
                try:
                    self.detection_pub.publish(detection_msg)
                except rospy.ROSException:
                    return

            if len(candidates) > 1:
                rospy.loginfo_throttle(
                    2.0,
                    '%s 有 %d 个目标，选用最近 depth=%.2fm（忽略 %.2fm）',
                    color_name, len(candidates), nearest['depth'],
                    max(d['depth'] for d in candidates))

            rospy.loginfo_throttle(
                2.0,
                '看到 %s 箱子(最近): camera=(x %.3f, depth %.2f)',
                color_name, cam_x, depth_meters)

            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
            label = '%s %.2fm 近' % (color_name, depth_meters)
            cv2.putText(cv_image, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.circle(cv_image, (hd_cx, hd_cy), 5, (0, 0, 255), -1)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            rospy.logwarn_throttle(10.0, '发布 debug_image 失败: %s', e)

        if self.show_opencv_window:
            cv2.imshow('Kinect Vision - Box Detection', cv_image)
            cv2.waitKey(1)

    def run(self):
        def shutdown_hook():
            rospy.loginfo('视觉节点关闭中...')
            cv2.destroyAllWindows()

        rospy.on_shutdown(shutdown_hook)
        rospy.spin()


if __name__ == '__main__':
    try:
        vision_node = RealVisionNode()
        vision_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('视觉节点被用户中断')
    except Exception as e:
        rospy.logerr('视觉节点发生未预期错误: %s', e)
        import traceback
        rospy.logerr(traceback.format_exc())
    finally:
        cv2.destroyAllWindows()
