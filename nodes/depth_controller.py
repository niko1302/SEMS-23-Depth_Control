#!/usr/bin/env python3
"""
This node is your depth controller.
It takes as input a current depth and a given depth setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""
import rclpy
from hippo_msgs.msg import ActuatorSetpoint, DepthStamped, Float64Stamped
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import time


class DepthControlNode(Node):

    def __init__(self):
        super().__init__(node_name='depth_controller')

        # -------------------------------------------
        # -- Declare all parameters from yaml file --
        # -------------------------------------------
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gains.p', rclpy.Parameter.Type.DOUBLE),
                ('gains.d', rclpy.Parameter.Type.DOUBLE),
                ('gains.i', rclpy.Parameter.Type.DOUBLE),
                ('toggle_i_controller', rclpy.Parameter.Type.BOOL),
                ('near_setpoint_value', rclpy.Parameter.Type.DOUBLE),
            ]
        )

        param = self.get_parameter('gains.p')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.p_gain = param.value

        param = self.get_parameter('gains.i')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.i_gain = param.value

        param = self.get_parameter('gains.d')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.d_gain = param.value

        param = self.get_parameter('toggle_i_controller')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.toggle_i_controller = param.value

        param = self.get_parameter('near_setpoint_value')
        self.get_logger().info(f'{param.name} = {param.value}')
        self.near_setpoint_value = param.value

        self.add_on_set_parameters_callback(self.on_params_changed)

        # -----------------------------
        # -- Set Class-wide Varables --
        # -----------------------------
        
        # -- Creates a publisher for the depth error --
        self.publish_depth_error = True
        self.publish_thrust = True

        # -- Toggle logger informations --
        self.log = False
        self.log_thrust_min_max = False
        self.log_controllers = False
        self.max_thrust = 0.0
        self.min_thrust = 0.0

        # -- Class-wide varaibles --
        self.current_setpoint = 0.0
        self.last_depth_msg = DepthStamped()
        self.last_error = 0.0
        self.i_error = 0.0
        self.ignore_first = True    # ignore first thrust calculation

        # ------------------------------------
        # -- Set Publishers and Subscribers --
        # ------------------------------------
        self.thrust_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                topic='thrust_setpoint',
                                                qos_profile=1)

        self.setpoint_sub = self.create_subscription(msg_type=Float64Stamped,
                                                     topic='depth_setpoint',
                                                     callback=self.on_setpoint,
                                                     qos_profile=1)
        self.depth_sub = self.create_subscription(msg_type=DepthStamped,
                                                  topic='depth',
                                                  callback=self.on_depth,
                                                  qos_profile=1)
        
        # -- publishers for the thrust and the individual P I & D parts --
        if self.publish_thrust:
            self.thrust_log_z = self.create_publisher(
                msg_type=Float64Stamped,
                topic='thrust_log_z',
                qos_profile=1)
            self.thrust_log_z_p = self.create_publisher(
                msg_type=Float64Stamped,
                topic='thrust_log_z_p',
                qos_profile=1)
            self.thrust_log_z_i = self.create_publisher(
                msg_type=Float64Stamped,
                topic='thrust_log_z_i',
                qos_profile=1)
            self.thrust_log_z_d = self.create_publisher(
                msg_type=Float64Stamped,
                topic='thrust_log_z_d',
                qos_profile=1)

        # -- Publisher for the depth error --
        if self.publish_depth_error:
            self.depth_error_pub = self.create_publisher(
                msg_type=Float64Stamped,
                topic='depth_error',
                qos_profile=1
            )


    def on_params_changed(self, params):
        param: rclpy.Parameter
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == 'gains.p':
                self.p_gain = param.value
            elif param.name == 'gains.i':
                self.i_gain = param.value
            elif param.name == 'gains.d':
                self.d_gain = param.value
            elif param.name == 'toggle_i_controller':
                self.toggle_i_controller = param.value
            elif param.name == 'near_setpoint_value':
                self.near_setpoint_value = param.value
            else:
                continue
        return SetParametersResult(succesful=True, reason='Parameter set')


    def on_setpoint(self, setpoint_msg: Float64Stamped):
        # We received a new setpoint! Let's save it, so that we can use it as
        # soon as we receive new depth data.
        self.current_setpoint = setpoint_msg.data


    def on_depth(self, depth_msg: DepthStamped):
        # We received a new depth message! Now we can get to action!

        # -- null commands if not -0.8 < depth < -0.1 --
        if depth_msg.depth > -0.1 or depth_msg.depth < -0.8:
            thrust = 0.0
            self.i_error = 0.0
        else:
            thrust = self.compute_control_output(depth_msg)
        
        timestamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        self.publish_vertical_thrust(thrust=thrust, timestamp=timestamp)


    def publish_vertical_thrust(self, thrust: float,
                                timestamp: rclpy.time.Time) -> None:
        msg = ActuatorSetpoint()
        # we want to set the vertical thrust exlusively. mask out xy-components.
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False

        msg.z = thrust

        # Let's add a time stamp
        msg.header.stamp = timestamp.to_msg()

        self.thrust_pub.publish(msg)


    def compute_control_output(self, depth_msg: DepthStamped) -> float:
        # -- Get current depth and time --
        current_depth = depth_msg.depth
        now = depth_msg.header.stamp.sec + (depth_msg.header.stamp.nanosec * 1e-9)

        # -- Calculate the depth error --
        error = self.current_setpoint - current_depth
        if self.publish_depth_error:
            error_msg = Float64Stamped()
            error_msg.data = error
            error_msg.header.stamp = self.get_clock().now().to_msg()
            self.depth_error_pub.publish(error_msg)

        # -- Don't execute on first run --
        if self.ignore_first:
            self.last_error = error             # Save the first error for next time
            self.last_depth_msg = depth_msg     # Save the first depth for next time
            self.ignore_first = False
            return 0.0

        # -- Get last depth and time --
        # last_depth = self.last_depth_msg.depth
        last_time = self.last_depth_msg.header.stamp.sec + (self.last_depth_msg.header.stamp.nanosec * 1e-9)
        d_time = now - last_time

        # -- Calculate derivative of error --
        d_error = (error - self.last_error) / d_time

        # -- Calculate the integral of error --
        # i_error = 0 wenn lange kein Signal (normal 50Hz)
        if (self.toggle_i_controller and abs(error) > self.near_setpoint_value) or d_time > 1.0:
            self.i_error = 0.0
        else:
            average_error = (self.last_error + error) / 2
            self.i_error += average_error * d_time

        # -- Calculate thrust --
        p_control = self.p_gain * error
        d_control = self.d_gain * d_error
        i_control = self.i_gain * self.i_error
        thrust_z = p_control + d_control + i_control

        # -- thrust_z must be a value betweed -1 and +1 --
        if thrust_z > 1.0: thrust_z = 1.0       # Thrust can't be greater than +1
        if thrust_z < -1.0: thrust_z = -1.0     # Thrust can't be smaller than -1

        # -- Logger Infos --
        if self.log:
            self.get_logger().info(
                f'I received a depth of {current_depth} m.',
                throttle_duration_sec=1)
            self.get_logger().info(
                f'I received an error of {error} m.',
                throttle_duration_sec=1)
        if self.log_thrust_min_max:
            if thrust_z > self.max_thrust: self.max_thrust = thrust_z
            if thrust_z < self.min_thrust: self.min_thrust = thrust_z
            self.get_logger().info(
                f'Thrust (max,min) = ({self.max_thrust}, {self.min_thrust})')
        if self.log_controllers:
            self.get_logger().info(
                f'error = {error:.5f}; Derivation = {d_error:.5f}; Integral = {i_control:.5f}')
        if self.publish_thrust:
            thrust_log = Float64Stamped()
            thrust_log.header.stamp = self.get_clock().now().to_msg()
            thrust_log.data = thrust_z
            self.thrust_log_z.publish(thrust_log)
            thrust_log.data = p_control
            self.thrust_log_z_p.publish(thrust_log)
            thrust_log.data = i_control
            self.thrust_log_z_i.publish(thrust_log)
            thrust_log.data = d_control
            self.thrust_log_z_d.publish(thrust_log)

        # -- update saved Variables --
        self.last_error = error
        self.last_depth_msg = depth_msg

        return thrust_z


def main():
    rclpy.init()
    node = DepthControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
