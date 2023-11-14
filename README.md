# SEMS-23-Depth_Control

## Launch the Node

To Start the Node Type:

```
ros2 launch depth_control depth.launch.py vehicle_name:=<Vehicle Name>
```

The Launch Argument 'vehicle_name' has no deafult value and thus has to be set in the Terminal.

## Parameters

To list all current Parameters Type:
```
ros2 param list
```
Change a parameters default value, by editing the `.yaml` files in the `config/` folder.
To change the parameters during runtime enter:
```
ros2 param set <namespace.node> <parameter> <new_value>
```
Example: To Change the D-gain of the depth_controller Node from the Robot named "bluerov00":
```
ros2 param set /bluerov00/depth_controller gains.d 2.0
```
If you don't know the value of a parameter, type:
```
ros2 param get <namspace.node> <parameter>
```
