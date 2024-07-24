#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Altitude
from sensor_msgs.msg import FluidPressure as Pressure

WATER_DENSITY = 1000.0  # kg/m^3
GRAVITY = 9.81  # m/s^2


class Pressure2Depth(Node):
    def __init__(self):
        super().__init__("pressure2depth_node")

        self.declare_parameter("pressure_offset", 101325.0)
        self.pressure_offset = self.get_parameter("pressure_offset").value

        self.pressure_sub = self.create_subscription(
            Pressure, "bluerov2/pressure", self.pressure_callback, 10
        )

        self.depth_pub = self.create_publisher(Altitude, "bluerov2/depth", 10)

    def pressure_callback(self, msg):
        """
        Callback function for the pressure sensor.
        It converts the pressure to depth and publishes it.

        Args:
            msg (Pressure): The pressure message from the sensor
        """
        pressure = msg.fluid_pressure
        self.get_logger().debug(f"Pressure: {pressure:.2f} Pa")

        # Convert pressure (Pa) to depth (m) in water
        depth = (pressure - self.pressure_offset) / (WATER_DENSITY * GRAVITY)
        self.get_logger().debug(f"Depth: {depth:.2f} m")

        depth_msg = Altitude()
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.local = depth
        self.depth_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    depthPIDNode = Pressure2Depth()

    try:
        rclpy.spin(depthPIDNode)
    except KeyboardInterrupt:
        pass
    finally:
        depthPIDNode.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
