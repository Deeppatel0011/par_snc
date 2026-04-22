import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

class HazardDetector(Node):
    def __init__(self):
        super().__init__('hazard_detector_bridge')
        
        # Subscribes to the raw math data from find_object_2d
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/objects', 
            self.object_callback,
            10)
            
        # Publishes the clean string to your Node 2
        self.publisher = self.create_publisher(String, '/hazard_detections', 10)
        
        # Assignment Dictionary: Maps ID to Name
        self.hazard_map = {
            0: "Unknown",
            1: "Explosive",
            2: "Flammable Gas",
            3: "Non-Flammable Gas",
            4: "Dangerous When Wet",
            5: "Flammable Solid",
            6: "Spontaneously Combustible",
            7: "Oxidizer",
            8: "Organic Peroxide",
            9: "Inhalation Hazard",
            10: "Poison",
            11: "Radioactive",
            12: "Corrosive"
        }
        
        self.get_logger().info("Hazard Bridge Node Started.")
        self.get_logger().info("Ready to translate IDs to Hazard Names.")

    def object_callback(self, msg):
        # find_object_2d sends an array. The first number is the Object ID.
        if len(msg.data) > 0:
            obj_id = int(msg.data[0])
            
            # Look up the ID in our assignment dictionary
            if obj_id in self.hazard_map:
                hazard_name = self.hazard_map[obj_id]
                
                # Format: "ID,Name,CenterPixelX,CenterPixelY"
                # We use 320, 240 as default image centers for calculation
                detection_str = f"{obj_id},{hazard_name},320,240"
                
                output_msg = String()
                output_msg.data = detection_str
                
                self.publisher.publish(output_msg)
                self.get_logger().info(f"DETECTED: ID {obj_id} is {hazard_name}")
            else:
                self.get_logger().warn(f"Detected ID {obj_id}, but it's not in the assignment list!")

def main(args=None):
    rclpy.init(args=args)
    node = HazardDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()