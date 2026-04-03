#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped

from cv_bridge import CvBridge
import numpy as np
import cv2

from message_filters import Subscriber, ApproximateTimeSynchronizer
from ultralytics.utils.plotting import colors

import tf2_ros
import tf2_geometry_msgs  # registers PointStamped with tf2; must be imported even if unused directly
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


CLASS_NAMES = {
    "0":  "left_gate_pole",
    "1":  "right_gate_pole",
    "2":  "shark",
    "3":  "sawfish",
    "4":  "drop_box",
    "5":  "red_buoy",
    "6":  "red_pole",
    "7":  "white_pole",
    "8":  "path_marker",
    "9":  "octagon",
    "10": "table",
    "11": "ladle",
    "12": "bottle",
}

FONT           = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE     = 0.5
FONT_THICKNESS = 1
COLOR_WHITE    = (255, 255, 255)
COLOR_BLACK    = (0, 0, 0)
COLOR_GREEN    = (0, 255, 0)


def draw_detection(img, xmin, ymin, xmax, ymax, u_obj, v_obj,
                   name, color, z, x_odom, y_odom, z_odom):
    """
    Draws all per-detection annotations onto the image in place.
    Keeps the label, depth, centroid, and coordinate overlay visually grouped.
    """
    # Bounding box
    cv2.rectangle(img, (xmin, ymin), (xmax, ymax), color, 2)

    # Class label + depth above the box
    label = f"{name}  {z:.2f}m"
    (lw, lh), _ = cv2.getTextSize(label, FONT, FONT_SCALE, FONT_THICKNESS)
    label_y = max(ymin - 6, lh + 6)
    cv2.rectangle(img, (xmin, label_y - lh - 4), (xmin + lw + 6, label_y + 4), color, -1)
    cv2.putText(img, label, (xmin + 3, label_y), FONT, FONT_SCALE, COLOR_BLACK, FONT_THICKNESS)

    # Centroid dot
    cv2.circle(img, (u_obj, v_obj), 4, COLOR_GREEN, -1)

    # Coordinate overlay — shown as a small pill anchored to the centroid
    coord_lines = [
        f"x {x_odom:+.2f}",
        f"y {y_odom:+.2f}",
        f"z {z_odom:+.2f}",
    ]
    line_h      = 18
    padding     = 6
    box_w       = 80
    box_h       = line_h * len(coord_lines) + padding * 2

    # Anchor the box to the right of the centroid, nudge it inward if it would clip the edge
    bx = min(u_obj + 10, img.shape[1] - box_w - 4)
    by = max(v_obj - box_h // 2, 4)
    by = min(by, img.shape[0] - box_h - 4)

    # Semi-transparent dark background
    overlay = img.copy()
    cv2.rectangle(overlay, (bx, by), (bx + box_w, by + box_h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)

    # Thin border in the detection color
    cv2.rectangle(img, (bx, by), (bx + box_w, by + box_h), color, 1)

    for i, line in enumerate(coord_lines):
        ty = by + padding + (i + 1) * line_h - 4
        cv2.putText(img, line, (bx + padding, ty), FONT, 0.42, COLOR_WHITE, 1)


class ROIDepthFusion(Node):
    """
    Subscribes to synchronized RGB, depth, and 2D detection streams.
    For each detection, crops the depth ROI, estimates the object's 3D
    position via pinhole projection, transforms it into the odom frame,
    and publishes a Detection3DArray. Also publishes an annotated image
    for visualization in RViz.
    """

    def __init__(self):
        super().__init__('roi_depth_fusion_node')

        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        self.declare_parameter('min_depth', 0.2)
        self.declare_parameter('max_depth', 20.0)
        self.declare_parameter('min_valid_pixels', 30)

        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.min_valid = self.get_parameter('min_valid_pixels').value

        self.bridge      = CvBridge()
        self.camera_info = None

        # 10s TF cache to handle cases where the transform arrives slightly late
        self.tf_buffer   = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.depth_sub = Subscriber(self, Image, '/camera/depth_image_raw/front')
        self.rgb_sub   = Subscriber(self, Image, '/camera/RGB_image_raw/front')
        self.det_sub   = Subscriber(self, Detection2DArray, '/detections_2d')

        self.info_sub = self.create_subscription(
            CameraInfo, '/camera_info_front', self.camera_info_cb, 10
        )

        # slop=0.2s tolerates the typical latency between camera capture and detector output
        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.rgb_sub, self.det_sub],
            queue_size=10,
            slop=0.2,
        )
        self.sync.registerCallback(self.synced_cb)

        self.pub = self.create_publisher(Detection3DArray, '/detections_3d', 10)
        self.viz_pub = self.create_publisher(
            Image,
            '/depth_fusion/detection_image',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )

        self.get_logger().info("ROI depth fusion node started.")

    # ------------------------------------------------------------------

    def camera_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def synced_cb(self, depth_msg: Image, rgb_msg: Image, det_msg: Detection2DArray):
        if self.camera_info is None:
            self.get_logger().warn("Camera info not yet received — skipping frame.")
            return

        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        rgb_img   = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        out        = Detection3DArray()
        out.header = det_msg.header

        self.get_logger().info(f"Processing {len(det_msg.detections)} detections.")

        for det in det_msg.detections:
            if not det.results:
                continue

            hyp      = det.results[0].hypothesis
            class_id = hyp.class_id
            score    = hyp.score
            bbox     = det.bbox

            u = int(bbox.center.position.x)
            v = int(bbox.center.position.y)
            w = int(bbox.size_x)
            h = int(bbox.size_y)

            xmin = max(u - w // 2, 0)
            xmax = min(u + w // 2, depth_img.shape[1] - 1)
            ymin = max(v - h // 2, 0)
            ymax = min(v + h // 2, depth_img.shape[0] - 1)

            roi        = depth_img[ymin:ymax, xmin:xmax]
            valid_mask = np.isfinite(roi) & (roi > self.min_depth) & (roi < self.max_depth)

            if np.count_nonzero(valid_mask) < self.min_valid:
                continue

            # Histogram over valid depths to isolate the foreground cluster.
            # Without this, background pixels inside the bbox skew the depth estimate.
            valid_depths = roi[valid_mask]
            hist, bins   = np.histogram(valid_depths, bins=50)
            peak_idx     = np.argmax(hist)
            bin_center   = (bins[peak_idx] + bins[peak_idx + 1]) / 2.0
            fg_mask      = valid_mask & (roi >= bin_center - 0.3) & (roi <= bin_center + 0.3)

            if np.count_nonzero(fg_mask) < self.min_valid:
                continue

            ys, xs = np.where(fg_mask)
            u_obj  = int(np.mean(xs)) + xmin
            v_obj  = int(np.mean(ys)) + ymin

            # Median is more robust than min here — a single noisy foreground pixel
            # won't pull the depth estimate closer than the actual object surface.
            z = float(np.median(roi[fg_mask]))
            x = (u_obj - cx) * z / fx
            y = (v_obj - cy) * z / fy

            point_camera                 = PointStamped()
            point_camera.header.frame_id = 'zed_camera_front_link_optical_frame'
            point_camera.header.stamp    = depth_msg.header.stamp
            point_camera.point.x         = x
            point_camera.point.y         = y
            point_camera.point.z         = z

            try:
                point_odom = self.tf_buffer.transform(point_camera, 'odom', timeout=Duration(seconds=1.0))
                x_odom = point_odom.point.x
                y_odom = point_odom.point.y
                z_odom = point_odom.point.z
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                # TF tree not yet populated or extrapolation window missed — use camera frame as fallback
                self.get_logger().debug(f"TF transform failed: {e} — publishing in camera frame.")
                x_odom, y_odom, z_odom = x, y, z

            name  = CLASS_NAMES.get(class_id, class_id)
            color = colors(int(class_id), True)

            draw_detection(rgb_img, xmin, ymin, xmax, ymax,
                           u_obj, v_obj, name, color,
                           z, x_odom, y_odom, z_odom)

            det3d        = Detection3D()
            det3d.header = det.header

            hyp3d                     = ObjectHypothesisWithPose()
            hyp3d.hypothesis.class_id = class_id
            hyp3d.hypothesis.score    = score

            pose             = Pose()
            pose.position    = Point(x=x_odom, y=y_odom, z=z_odom)
            pose.orientation = Quaternion(w=1.0)
            hyp3d.pose.pose  = pose

            det3d.results.append(hyp3d)
            det3d.bbox.center = pose
            det3d.bbox.size.x = 0.1
            det3d.bbox.size.y = 0.1
            det3d.bbox.size.z = 0.1

            out.detections.append(det3d)

        self.pub.publish(out)

        ros_img        = self.bridge.cv2_to_imgmsg(rgb_img, encoding='bgr8')
        ros_img.header = rgb_msg.header
        self.viz_pub.publish(ros_img)
        cv2.imshow("Depth Fusion Detections", rgb_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ROIDepthFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()