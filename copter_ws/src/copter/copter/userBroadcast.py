import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from example_interfaces.msg import UInt16


#! this broadcaster is for testing only, will be restructured to use a custom interface

class OLTester(Node):
    def __init__(self):
        super().__init__("CopterTester")

        self._publisher = self.create_publisher(
            UInt16,
            '/manualSetpoint',
            10
        )

def main(args = None):
    rclpy.init(args=args)
    node = OLTester()
    try:
        while rclpy.ok():

            value = input("Duty Cycle > ")
            msg = UInt16()
            msg.data = int(value)

            node._publisher.publish(msg)

            node.get_logger().info(f"Published PWM: {msg.data}")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
