# SEMS-23-Depth_Control

## Launch the Node

To Start the Node Type:

```
ros2 launch depth_control depth.launch.py vehicle_name:=<Vehicle Name>
```

The Launch Argument 'vehicle_name' has no deafult value and thus has to be set in the Terminal.

## Parameters

### Change and List Paramters

To list all current Parameters in the Terminal type:
```
ros2 param list
```

Change a parameters default value, by editing the `.yaml` files in the `config/` folder.
To change the parameters during runtime enter:
```
rqt
```
and navigate to "Plugins" -> "Configuration" -> "Dynamic Reconfigure"

### Calculator Parameters

The `depth_calculator` Node has the following parameters:

* `pressure_atmospheric' [DOUBLE] is the pressure caused by the atmosphere at sea level at the time, the robot is being used. This pressure is measured with a reference device, so the robots sensor data doesn't have to be read every time. *The default value is 101325 Pa which is the air pressure at sea level at 20°C *
* `pressure_offset` [DOUBLE] is the pressure offset from the robots sensor and the reference device used to measure the atmosphere pressure. *The default value is 0*
* `depth_offset` [DOUBLE] is offset of the Sensors position compared to the robots origin. *The default value is 0*
* `rho_water` [DOUBLE] is the density of water (the fluid the robot is moving in) *The default value is 998.2 kg/m³ which is the density of water at atmospheric pressure and 20°C*
* `signal_smoothing` [BOOL] toggles the smoothing (noise reduction) of the pressure data, by using a simple moving average filter. It is recommended to use this when using real data. *The default value is True so the signal is being smoothed*
* `moving_average_value` [DOUBLE] sets the value for the moving average filter. *The default is 0.3*

### Controller Parameters

The `depth_controller` Node hast the following parameters:

* `gains.p` [DOUBLE] : P-Gain *default value is 5.0*
* `gains.i` [DOUBLE] : I-Gain *default value is 2.0*
* `gains.d` [DOUBLE] : D-Gain *dafault value is 4.0*
* `toggle_i_controller` [BOOL] shuts the I-Controller off, if the robot is far away from the setpoint. This will reduce the overshoot, caused by the I-Controller, while keeping the steady-state-error near 0. *The default value is True, which enables the described behavior*
* `near_setpoint_value` [DOUBLE] sets the point at which the I-Controller will be shut off, if enabled *The default value is 0.03m*

### Setpoint Publisher Parameters

The `depth_setpoint_publisher` Node has the following parameters:


