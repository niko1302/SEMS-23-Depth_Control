#!/usr/bin/env python3
"""
This node takes as input the pressure data and computes a resulting water depth.
"""
import rclpy
from hippo_msgs.msg import DepthStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import FluidPressure

from scipy import constants


class DepthCalculator(Node):

    def __init__(self):
        super().__init__(node_name='depth_calculator')

        # -------------------------------------------
        # -- Declare all parameters from yaml file --
        # -------------------------------------------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pressure_atmospheric', rclpy.Parameter.Type.DOUBLE),
                ('pressure_offset', rclpy.Parameter.Type.DOUBLE),
                ('depth_offset', rclpy.Parameter.Type.DOUBLE),
                ('rho_water', rclpy.Parameter.Type.DOUBLE)
            ]
        )

        param = self.get_parameter('pressure_atmospheric')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.pressure_atmospheric = param.value
        
        param = self.get_parameter('pressure_offset')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.pressure_offset = param.value

        param = self.get_parameter('depth_offset')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.depth_offset = param.value

        param = self.get_parameter('rho_water')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.rho_water = param.value

        # TODO parameter change function
        
        # ------------------------------------
        # -- Set Publishers and Subscribers --
        # ------------------------------------
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        self.depth_pub = self.create_publisher(msg_type=DepthStamped,
                                               topic='depth',
                                               qos_profile=1)
        self.pressure_sub = self.create_subscription(msg_type=FluidPressure,
                                                     topic='pressure',
                                                     callback=self.on_pressure,
                                                     qos_profile=qos)

    def on_pressure(self, pressure_msg: FluidPressure) -> None:
        pressure = pressure_msg.fluid_pressure
        depth = self.pressure_to_depth(pressure=pressure)
        now = self.get_clock().now()
        self.publish_depth_msg(depth=depth, now=now)

    def publish_depth_msg(self, depth: float, now: rclpy.time.Time) -> None:
        msg = DepthStamped()
        # Let's add a time stamp
        msg.header.stamp = now.to_msg()
        # and populate the depth field
        msg.depth = depth
        self.depth_pub.publish(msg)

    def pressure_to_depth(self, pressure: float) -> float:
        # ----- Calculation -----
        # p_w = rho * g * h  ==>  h = p_w / (rho * g)
        # h = - pos_z  ==> pos_z = - p_w / (rho * g)
        # p = p_w + p_0  ==>  pos_z = - (p - p_0) / (rho * g)
        # -----------------------

        # -- Calculate the waterpressure pw --
        pressure_water = pressure - self.pressure_atmospheric + self.pressure_offset
        depth = pressure_water / (self.rho_water * constants.g)
        return - depth + self.depth_offset


def main():
    rclpy.init()
    node = DepthCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
