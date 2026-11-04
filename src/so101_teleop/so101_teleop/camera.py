"""camera -- USB wrist camera as a ROS image stream.

Interface:
  Publishes   /camera/image_raw             sensor_msgs/Image, encoding "bgr8"
              /camera/image_raw/compressed  sensor_msgs/CompressedImage, format "jpeg" (optional)
Parameters:   index 0 | width 640 | height 480 | fps 30 | frame_id "wrist_camera"

Windows tips (each of these cost someone an afternoon):
  - open with cv2.VideoCapture(index, cv2.CAP_DSHOW): the default backend can take 10+ s to open
  - set cv2.CAP_PROP_BUFFERSIZE to 1 so you publish the newest frame, not a queued old one
  - Image.data is uint8[]: assign array("B", frame.tobytes()) (from the `array` module).
    Assigning plain bytes works but rclpy then type-checks every byte in Python: ~50 ms per frame.
  - 640x480 like real_imitation: 1080p over USB 2.0 slows the servo bus on the same hub
View it in rviz2 (Add > By topic > /camera/image_raw > Image).
"""

import rclpy
from rclpy.node import Node


class Camera(Node):
    def __init__(self):
        super().__init__("camera")
        # TODO(you), milestone 6: parameters, open the camera (fail loudly if not isOpened()),
        # publisher(s), timer at fps -> self.loop
        raise NotImplementedError("Camera.__init__")

    def loop(self):
        # TODO(you): read a frame, fill sensor_msgs/Image (height, width, encoding, step = width*3,
        # data, header.stamp, header.frame_id), publish. Optionally also a JPEG via cv2.imencode.
        pass


def main():
    rclpy.init()
    node = Camera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
