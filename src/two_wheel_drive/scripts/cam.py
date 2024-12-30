import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # Initialize YOLO model
        self.model = YOLO('last.pt')  # Load your trained YOLO model
        self.target_class = 1  # Replace with the correct class index for "Drivable-Area"

        # Create a subscription to the image topic
        self.subscription = self.create_subscription(
            ROSImage,
            '/depth_camera/image_raw',
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.get_logger().info("YOLO Node Initialized and Subscribed to /camera/image_raw")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to RGB for YOLO inference
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Run YOLO inference
            results = self.model(rgb_image)
            result = results[0]

            # Get masks and associated classes
            masks = result.masks.data if result.masks else []
            classes = result.boxes.cls if result.boxes else []

            # Extract mask(s) for the target class
            drivable_area_masks = [masks[i] for i, cls in enumerate(classes) if int(cls) == self.target_class]

            if drivable_area_masks:
                # Combine masks and create an overlay
                combined_mask = np.any([mask.cpu().numpy() for mask in drivable_area_masks], axis=0)
                binary_mask = (combined_mask * 255).astype('uint8')
                binary_mask_resized = cv2.resize(binary_mask, (cv_image.shape[1], cv_image.shape[0]))
                
                # Highlight the drivable area in green
                # overlay = cv_image.copy()
                #overlay[binary_mask_resized > 0] = [0, 255, 0]
                #output_image = cv2.addWeighted(overlay, 0.5, cv_image, 0.5, 0)
                output_image=binary_mask_resized
            else:
                output_image = cv_image

            # Display the output
            cv2.imshow("Drivable Area Detection", output_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
