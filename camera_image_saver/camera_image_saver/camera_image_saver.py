import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
        def __init__(self , name):
                    super(). __init__(name)
                    self.get_logger().info("Node created")
                    self.subscription = self.create_subscription(
                        Image,
                        '/rgb_camera/image_raw',
                        self.image_callback,
                        10  # QoS depth
                    )
                    self.bridge = CvBridge()
                    self.frame_count = 0  # the name of image will start from 0

                    # create directory
                    home_dir = os.path.expanduser("~")  
                    self.image_save_path = os.path.join(home_dir, "camera_images")
                    if not os.path.exists(self.image_save_path):
                        os.makedirs(self.image_save_path)
                    self.get_logger().info(f"Images will be saved to: {self.image_save_path}")

        def image_callback(self, msg):
            try:
                # ROS2 image message to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
                # save the images
                image_filename = os.path.join(self.image_save_path, f"frame_{self.frame_count:04d}.png")
                cv2.imwrite(image_filename, cv_image)
                self.get_logger().info(f"Saved image: {image_filename}")
                
                self.frame_count += 1
            except Exception as e:
                self.get_logger().error(f"Failed to save image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    image_saver_node = ImageSaver("camera_image_saver")
    try:
            rclpy.spin(image_saver_node)
    except KeyboardInterrupt:
            pass
    image_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  