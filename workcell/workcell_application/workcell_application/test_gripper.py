import rclpy
from rclpy.node import Node
from ur_msgs.srv import SetIO
import time

def main():
    rclpy.init()
    node = rclpy.create_node('io_gripper_test')
    
    # Verbinde mit dem Standard-IO-Controller von UR
    io_client = node.create_client(SetIO, '/io_and_status_controller/set_io')
    
    while not io_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Warte auf IO-Service...')

    req = SetIO.Request()
    req.fun = 1  # 1 bedeutet: Standard Digital Output
    req.pin = 0  # Wir nutzen digital_out[0]

    # 1. VAKUUM AN (Schalter = True)
    node.get_logger().info("🟢 GRIP: Setze digital_out[0] = ON")
    req.state = 1.0
    io_client.call_async(req)
    
    time.sleep(4.0)

    # 2. VAKUUM AUS (Schalter = False)
    node.get_logger().info("🔴 RELEASE: Setze digital_out[0] = OFF")
    req.state = 0.0
    io_client.call_async(req)

    time.sleep(1.0)
    node.destroy_node()
    if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()