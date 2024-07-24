#!/usr/bin/env python

from time import sleep

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ArmingTimerNode(Node):
    done = False

    def __init__(self):
        super().__init__("arming_timer")

        self.declare_parameter("arming_timeout", 360.0)
        self.arming_timeout = self.get_parameter("arming_timeout").value

        self.arming_client = self.create_client(SetBool, "bluerov2/arming")

        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("BlueROV2 arming service not available, waiting...")

    def arm(self):
        """
        Arm the BlueROV2 and sleep for arming_timeout seconds
        """
        # Arm
        arm_future = self._arm()
        rclpy.spin_until_future_complete(self, arm_future)
        self.get_logger().info("Armed!")

        # Sleep for arming_timeout seconds
        sleep(self.arming_timeout)

        # Disarm
        disarm_future = self._disarm()
        rclpy.spin_until_future_complete(self, disarm_future)
        self.get_logger().info("Disarmed!")

        self.done = True

    def _arm(self):
        self.get_logger().info(f"Arming...")
        future = self.arming_client.call_async(SetBool.Request(data=True))
        return future

    def _disarm(self):
        self.get_logger().info(f"Disarming...")
        future = self.arming_client.call_async(SetBool.Request(data=False))
        return future

    def destroy_node(self):
        """
        Disarm the BlueROV2 before shutting down the node
        """
        disarm_future = self._disarm()
        rclpy.spin_until_future_complete(self, disarm_future)
        self.get_logger().debug("Disarmed before shutting down the node")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    armingNode = ArmingTimerNode()

    try:
        armingNode.arm()

        while rclpy.ok():
            rclpy.spin_once(armingNode)
            if armingNode.done:
                break
    except KeyboardInterrupt:
        pass
    finally:
        armingNode.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
