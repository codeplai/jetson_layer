#!/usr/bin/env python3
"""MINEBOT-Q Camera Node — YOLOv8 ONNX inference on Jetson Orin NX."""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge

MINING_CLASSES = {0: 'person', 1: 'obstacle', 2: 'crack', 3: 'flood'}


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('fps', 15)
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('model_path',
                               '/home/unitree/cyclonedds_ws/models/yolov8n_mining_int8.onnx')
        self.declare_parameter('use_cuda', True)

        self.device_id = self.get_parameter('device_id').value
        self.fps = self.get_parameter('fps').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.model_path = self.get_parameter('model_path').value
        self.use_cuda = self.get_parameter('use_cuda').value

        self.bridge = CvBridge()
        self.session = self._load_model()
        self.cap = self._open_camera()

        self.pub_raw = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_detections = self.create_publisher(
            Detection2DArray, '/camera/detections', 10)
        self.pub_compressed = self.create_publisher(
            CompressedImage, '/camera/compressed', 10)

        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self._capture_callback)
        self.get_logger().info(
            f'CameraNode started — device={self.device_id}, fps={self.fps}, '
            f'cuda={self.session is not None and self.use_cuda}')

    def _load_model(self):
        try:
            import onnxruntime as ort
            providers = []
            if self.use_cuda:
                available = ort.get_available_providers()
                if 'CUDAExecutionProvider' in available:
                    providers.append('CUDAExecutionProvider')
                    self.get_logger().info('ONNX using CUDAExecutionProvider')
                else:
                    self.get_logger().warn(
                        'CUDA not available, falling back to CPUExecutionProvider')
            providers.append('CPUExecutionProvider')
            session = ort.InferenceSession(self.model_path, providers=providers)
            self.get_logger().info(f'Model loaded: {self.model_path}')
            return session
        except Exception as e:
            self.get_logger().error(f'Failed to load ONNX model: {e}')
            return None

    def _open_camera(self):
        cap = cv2.VideoCapture(self.device_id)
        if not cap.isOpened():
            self.get_logger().error(
                f'Cannot open camera device {self.device_id}')
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        return cap

    def _preprocess(self, frame):
        img = cv2.resize(frame, (640, 640))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)
        return img

    def _postprocess(self, output, orig_shape):
        detections = []
        if output is None or len(output) == 0:
            return detections

        preds = output[0]
        if preds.ndim == 3:
            preds = preds[0]

        # YOLOv8 output: (num_classes+4, num_boxes) transposed
        if preds.shape[0] < preds.shape[1]:
            preds = preds.T

        h_orig, w_orig = orig_shape[:2]
        scale_x = w_orig / 640.0
        scale_y = h_orig / 640.0

        for pred in preds:
            cx, cy, w, h = pred[0], pred[1], pred[2], pred[3]
            class_scores = pred[4:]
            class_id = int(np.argmax(class_scores))
            confidence = float(class_scores[class_id])

            if confidence < self.conf_thresh:
                continue
            if class_id not in MINING_CLASSES:
                continue

            det = Detection2D()
            det.bbox.center.x = float(cx * scale_x)
            det.bbox.center.y = float(cy * scale_y)
            det.bbox.size_x = float(w * scale_x)
            det.bbox.size_y = float(h * scale_y)

            hyp = ObjectHypothesisWithPose()
            hyp.id = MINING_CLASSES[class_id]
            hyp.score = confidence
            det.results.append(hyp)
            detections.append(det)

        return detections

    def _capture_callback(self):
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame capture failed')
            return

        now = self.get_clock().now().to_msg()

        # Publish raw image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = 'camera_link'
        self.pub_raw.publish(img_msg)

        # Publish compressed (JPEG 60%)
        comp_msg = CompressedImage()
        comp_msg.header.stamp = now
        comp_msg.header.frame_id = 'camera_link'
        comp_msg.format = 'jpeg'
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
        comp_msg.data = buf.tobytes()
        self.pub_compressed.publish(comp_msg)

        # Run inference
        if self.session is not None:
            input_tensor = self._preprocess(frame)
            input_name = self.session.get_inputs()[0].name
            output = self.session.run(None, {input_name: input_tensor})
            dets = self._postprocess(output, frame.shape)

            det_array = Detection2DArray()
            det_array.header.stamp = now
            det_array.header.frame_id = 'camera_link'
            det_array.detections = dets
            self.pub_detections.publish(det_array)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
