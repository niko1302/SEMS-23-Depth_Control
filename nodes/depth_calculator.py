#!/usr/bin/env python3
"""
This node takes as input the pressure data and computes a resulting water depth.
"""
import rclpy
from hippo_msgs.msg import DepthStamped, Float64Stamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import FluidPressure
from rcl_interfaces.msg import SetParametersResult

from custom_msgs.msg import FilteredPressure

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
                ('rho_water', rclpy.Parameter.Type.DOUBLE),
                ('signal_smoothing', rclpy.Parameter.Type.BOOL),
                ('moving_average_value', rclpy.Parameter.Type.DOUBLE),
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

        param = self.get_parameter('signal_smoothing')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.signal_smoothing = param.value

        param = self.get_parameter('moving_average_value')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.moving_average_value = param.value

        self.add_on_set_parameters_callback(self.on_params_changed)

        # --------------------------
        # -- Class-wide variables --
        # --------------------------

        # -- Save last pressure message (for signal smoothing) --
        self.last_pressure = self.pressure_atmospheric + self.pressure_offset
        
        # ------------------------------------
        # -- Set Publishers and Subscribers --
        # ------------------------------------
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        self.depth_pub = self.create_publisher(msg_type=DepthStamped,
                                               topic='depth',
                                               qos_profile=1)
        self.pressure_filtered_pub = self.create_publisher(msg_type=FilteredPressure,
                                                       topic='pressure_filtered',
                                                       qos_profile=1)
        self.pressure_sub = self.create_subscription(msg_type=FluidPressure,
                                                     topic='pressure',
                                                     callback=self.on_pressure,
                                                     qos_profile=qos)
        
    def on_params_changed(self, params):
        param: rclpy.Parameter
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == 'pressure_atmospheric':
                self.pressure_atmospheric =param.value
            elif param.name == 'pressure_offset':
                self.pressure_offset = param.value
            elif param.name == 'depth_offset':
                self.depth_offset = param.value
            elif param.name == 'rho_water':
                self.rho_water = param.value
            elif param.name == 'signal_smoothing':
                self.signal_smoothing = param.value
            elif param.name == 'moving_average_value':
                self.moving_average_value = param.value
            else:
                continue
        return SetParametersResult(succesful=True, reason='Parameter set')


    def on_pressure(self, pressure_msg: FluidPressure) -> None:
        pressure = pressure_msg.fluid_pressure
        now = self.get_clock().now()
        
        if self.signal_smoothing:
            # - Smooth the pressure data -
            pressure_filtered = self.moving_average_value * pressure + (1-self.moving_average_value) * self.last_pressure
            self.last_pressure = pressure_filtered
            depth = self.pressure_to_depth(pressure=pressure_filtered)
            # - Publish the filtered pressure data -
            self.publish_filtered_pressure(pressure_filtered=pressure_filtered, now=now)
        else:
            depth = self.pressure_to_depth(pressure=pressure)
        
        self.publish_depth_msg(depth=depth, now=now)


    def publish_filtered_pressure (self, pressure_filtered: float, now: rclpy.time.Time) -> None:
        msg = FilteredPressure()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pressure_filtered = pressure_filtered
        msg.signal_smoothing = self.signal_smoothing
        msg.moving_average_value = self.moving_average_value
        self.pressure_filtered_pub.publish(msg)


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
