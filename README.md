Moving Targets
==============

This repo contains ROS/Gazebo models and plugins for the purpose of moving lifelike objects in simulation.

## Usage ##

Note that the namespace `targets/<model name>` is hardcoded and that the parameters must be available within the `targets/<model name>` namespace in the `rosparam` server.

## Demo ##

To see it in action, run `roslaunch moving_targets test.launch`.

## Set up Trajectories ##

There are five types of trajectories that can be created: `goToPoint`, `waypoints`, `circle`, `ellipse`, and `random_walk`. You specify the type of trajectory and trajectory parameters in the roslaunch file. For
a complete example see `test.launch`. For information about each parameter read the descriptions below.

Notice that in `test.launch` that the waypoints used in the trajectories are relative, i.e., when spawning the mover in Gazebo, you may specify a non-zero starting location:
```xml
<!-- Spawn in Gazebo -->
<include file="$(find moving_targets)/launch/spawn_mover.launch">
    <arg name="name"    value="$(arg mover_name)" />
    <arg name="model"   value="$(find moving_targets)/models/moving_person_walking/model.sdf" />
    <arg name="x"       value="1"/>
    <arg name="y"       value="0"/>
    <arg name="z"       value="0"/>
    <arg name="yaw"     value="0"/>
</include>
```
The waypoints param used for defining the trajectory will be relative to this starting point.

### Parameters ###

| Name            | Description                                                                                        | Type        |
|-----------------|----------------------------------------------------------------------------------------------------|-------------|
| trajectory_type | Specifies the type of trajectory goToPoint : 0 waypoints: 1 circle: 2 ellipse : 3                  | int         |
| acceleration    | horizontal acceleration m/s/s                                                                      | float       |
| v               | horizontal velocity m/s                                                                            | float       |
| move_target     | indicates if the agent should be moving or not.                                                    | bool        |
| waypoints_x     | x-coordiante waypoints relative to starting position. First element must be 0.                     | float array |
| waypoints_y     | x-coordiante waypoints relative to starting position. First element must be 0.                     | float array |
| waypoints_z     | x-coordiante waypoints relative to starting position. First element must be 0.                     | float array |
| lambda          | The direction for circular orbit. The possible values are -1 (counter-clockwise) and 1 (clockwise).| int         |
| radius          | The radius of the orbit.                                                                           | float       |


### goToPoint ###

Moves the agent from the initial position to the desired position. Once reached, it will stop moving. 

Example setup

```xml
<group ns="targets/$(arg mover_name)">
    <param name="trajectory_type" type="int" value="0" />
    <param name="v" type="double" value="1" />
    <param name="move_target" type="bool" value="true"/>
    <rosparam param="waypoints_x">[0, 5]</rosparam>
    <rosparam param="waypoints_y">[0, 0]</rosparam>
    <rosparam param="waypoints_z">[0, 0]</rosparam>
</group>
```

### Waypoints ##

Moves the agent to the waypoints. Once at the final waypoint, it will loop through them again.

Example setup 

```xml
<group ns="targets/$(arg mover_name)">
    <param name="trajectory_type" type="int" value="1" />
    <param name="v" type="double" value="2" />
    <param name="move_target" type="bool" value="true"/>
    <rosparam param="waypoints_x">[0, 5, 5, 0, 0]</rosparam>
    <rosparam param="waypoints_y">[0, 0, 5, 5, 0]</rosparam>
    <rosparam param="waypoints_z">[0, 0, 0, 0, 0]</rosparam>
</group>
```

### Circle ##

Moves the agent in a circle. No waypoints needed.

Example setup

```xml
<group ns="targets/$(arg mover_name)">
    <param name="trajectory_type" type="int" value="2" />
    <param name="radius" type="double" value="3" />
    <param name="lambda" type="double" value = "-1"/>
    <param name="v" type="double" value="2" />
    <param name="move_target" type="bool" value="true"/>
</group>
```

### Elipse ###

Moves the agent in an elipse. Use only two waypoints. 

```xml
<group ns="targets/$(arg mover_name)">
    <param name="trajectory_type" type="int" value="3" />
    <param name="v" type="double" value="1" />
    <param name="move_target" type="bool" value="true"/>
    <rosparam param="waypoints_x">[0, 5]</rosparam>
    <rosparam param="waypoints_y">[0, 0]</rosparam>
    <rosparam param="waypoints_z">[0, 0]</rosparam>
</group>
```

### Constrained Random Walk ###

Moves the agent in a random walk within a bounding box specified by two waypoints.
The first waypoint is the bottom-left of the box while the second waypoint is the top-right.
The random walk is generated by sampling an `(x,y)` from two independent uniform distributions parameterized by the min and max as dictated by the bounding box.
Note that even the starting position of the mover is randomized within this bounding box---when spawning the mover as described in `test.launch`, the origin listed for the mover becomes the origin of the bounding box.

```xml
<group ns="targets/$(arg mover_name)">
    <param name="trajectory_type" type="int" value="4" />
    <param name="v" type="double" value="1" />
    <param name="move_target" type="bool" value="true"/>
    <rosparam param="waypoints_x">[0, 5]</rosparam>
    <rosparam param="waypoints_y">[0, 0]</rosparam>
    <rosparam param="waypoints_z">[0, 0]</rosparam>
</group>
```

## Use Plugin ##

The plugin has several parameters that will need to be set. 


| Name         | Description                                | Type  |
|--------------|--------------------------------------------|-------|
| kpPsi        | Heading proportional gain for PD control   | float |
| kpZ          | Altitude proportional gain for PD control  | float |
| kdPsid       | Heading differential gain for PD control   | float |
| kdZ          | Altitude differential gain for PD control  | float |
| maxVPsi      | Maximum velocity about z axis              | float |
| maxVZ        | Maximum linear velocity parallel to z axis | float |
| k_orbit      | Gain for orbital trajectory                | float |
| k_path       | Gain for linear trajectory                 | float |
| chi_infinity | Gain for linear trajectory                 | float |
| update_rate  | The target update rate in Hz               | float |

Example setup

```xml
<plugin name="TargetMotion" filename="libTargetMotion.so">
  <kpPsi>2</kpPsi>
  <kpZ>2</kpZ>      
  <kdPsi>1</kdPsi>
  <kdZ>1</kdZ>
  <maxVPsi>1</maxVPsi>
  <maxVZ>1</maxVZ>
  <k_orbit>2</k_orbit>
  <k_path>3</k_path>
  <chi_infinity>0.7853975</chi_infinity>
  <update_rate>30</update_rate>
</plugin>
```

## Exposed Services ##

The plugin advertises a ROS service that allows the user to reset the agent at any time, or to tell the agent to stop or move. The service is defined in `srv/MovingTargets.srv`.