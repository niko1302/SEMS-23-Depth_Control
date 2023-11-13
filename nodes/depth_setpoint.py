#!/usr/bin/env python3
"""
This node computes a square-wave setpoint for the depth controller, i.e.
the setpoint jumps between two different depth values with a set duration.
You can change this code to try out other setpoint functions, e.g. a sin wave.
"""
import rclpy
from hippo_msgs.msg import Float64Stamped
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import math


class DepthSetpointNode(Node):

    def __init__(self):
        super().__init__(node_name='depth_setpoint_publisher')

        # -------------------------------------------
        # -- Declare all parameters from yaml file --
        # -------------------------------------------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('setpoint_function', rclpy.Parameter.Type.STRING),
                ('func_period_duration', rclpy.Parameter.Type.DOUBLE),
                ('func_max_value', rclpy.Parameter.Type.DOUBLE),
                ('func_min_value', rclpy.Parameter.Type.DOUBLE),
            ]
        )

        param = self.get_parameter('setpoint_function')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.setpoint_function = param.value

        param = self.get_parameter('func_period_duration')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.func_half_period_duration = param.value / 2    # [s]

        param = self.get_parameter('func_max_value')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.func_max_value = param.value   # [m]

        param = self.get_parameter('func_min_value')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.func_min_value = param.value   # [m]

        # -- If a Parameter changes call this function --
        self.add_on_set_parameters_callback(self.on_params_changed)

        # -----------------------------
        # -- Set Class-wide Varables --
        # -----------------------------
        self.start_time = self.get_clock().now()

        # -- Calculations for functions (do not change) --
        self.set_func_variables()

        # ------------------------------------
        # -- Set Publishers and Subscribers --
        # ------------------------------------
        self.depth_setpoint_pub = self.create_publisher(msg_type=Float64Stamped,
                                                        topic='depth_setpoint',
                                                        qos_profile=1)
        self.timer = self.create_timer(timer_period_sec=1 / 50,
                                       callback=self.on_timer)


    def set_func_variables(self):
        self.func_amplitude = ((self.func_max_value - self.func_min_value)/2)    # [m]
        self.func_offset = ((self.func_max_value + self.func_min_value)/2)       # [m]
        self.func_linear_gradiant = (self.func_max_value - self.func_min_value) / self.func_half_period_duration # [m/s]


    def on_params_changed(self, params):
        param: rclpy.Parameter
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == 'setpoint_function':
                self.setpoint_function = param.value
                if not (param.value == 'rectangle' or
                        param.value == 'triangle' or
                        param.value == 'sin' or
                        param.value == 'const'):
                    self.get_logger().info(f'{param.name} not recognised. Setting constant function.')
            elif param.name == 'func_period_duration':
                self.func_half_period_duration = param.value / 2
                self.set_func_variables()
            elif param.name == 'func_max_value':
                self.func_max_value = param.value   # [m]
                self.set_func_variables()
            elif param.name == 'func_min_value':
                self.func_min_value = param.value
                self.set_func_variables()
            else:
                continue
        return SetParametersResult(succesful=True, reason='Parameter set')


    def on_timer(self) -> None:
        # change this for other setpoint functions
        now = self.get_clock().now()
        time = self.start_time - now
        
        # -- Rectangle Function --
        if self.setpoint_function == 'rectangle':   
            i = time.nanoseconds * 1e-9 % (self.func_half_period_duration * 2)
            if i > (self.func_half_period_duration):
                setpoint = self.func_max_value
            else:
                setpoint = self.func_min_value

        # -- Sinus Function --
        elif self.setpoint_function == 'sin':
            i = time.nanoseconds * 1e-9
            setpoint = self.func_amplitude * math.sin((math.pi/self.func_half_period_duration)*i) + self.func_offset

        # -- Triangle Function --
        elif self.setpoint_function == 'triangle':
            i = time.nanoseconds * 1e-9 % (self.func_half_period_duration * 2)
            if i < (self.func_half_period_duration):
                setpoint = self.func_linear_gradiant * i + self.func_min_value
            else:
                setpoint = -self.func_linear_gradiant * (i - self.func_half_period_duration) + self.func_max_value

        # -- Constant Function --
        else:
            setpoint = self.func_offset

        now = self.get_clock().now()
        self.publish_setpoint(setpoint=setpoint, now=now)

    def publish_setpoint(self, setpoint: float, now: rclpy.time.Time) -> None:
        msg = Float64Stamped()
        msg.data = setpoint
        msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_setpoint_pub.publish(msg)


def main():
    rclpy.init()
    node = DepthSetpointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
