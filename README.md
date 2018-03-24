Moving Targets
==============

This repo contains ROS/Gazebo models and plugins for the purpose of moving lifelike objects in simulation.

## Usage ##

Note that the namespace `targets/<model name>` is hardcoded and that the parameters must be available within the `targets/<model name>` namespace in the `rosparam` server.

## Set up Trajectories ##

There are four types of trajectories that can be created: goToPoint, waypoints, circle, and elipse. You specify the type of trajectory and trajectory parameters in the roslaunch file. For
a complete example see test.launch. For information about each parameter read the descriptions below. 

### Parameters ###

| Name            | Description                                                                                        | Type        |
|-----------------|----------------------------------------------------------------------------------------------------|-------------|
| trajectory_type | Specifies the type of trajectory goToPoint : 0 waypoints: 1 circle: 2 elipse : 3                   | int         |
| x               | initial x position in gazebo                                                                       | float       |
| y               | initial y position in gazebo                                                                       | float       |
| z               | initial z position in gazebo                                                                       | float       |
| v               | horizontal velocity m/s                                                                            | float       |
| move_target     | indicates if the agent should be moving or not.                                                    | bool        |
| waypoints_x     | x-coordiante waypoints relative to starting position. First element must be 0 .                    | float array |
| waypoints_y     | x-coordiante waypoints relative to starting position. First element must be 0 .                    | float array |
| waypoints_z     | x-coordiante waypoints relative to starting position. First element must be 0 .                    | float array |
| lambda          | The direction for circular orbit. The possible values are -1 (counter-clockwise) and 1 (clockwise) | int         |
| radius          | The radius of the orbit.                                                                           | float       |


### goToPoint ###

Moves the agent from the initial position to the desired position. Once reached, it will stop moving. 

Example setup

``` 
    <param name="trajectory_type" type="int" value="0" />
    <param name="x" type="double" value="1" />
    <param name="y" type="double" value="0" />
    <param name="z" type="double" value="0.6"/>
    <param name="v" type="double" value="1" />
    <param name="move_target" type="bool" value="true"/>
    <rosparam param="waypoints_x">[0, 5]</rosparam>
    <rosparam param="waypoints_y">[0, 0]</rosparam>
    <rosparam param="waypoints_z">[0, 0]</rosparam>
```

### Waypoints ##

Moves the agent to the waypoints. Once at the final waypoint, it will loop through them again.

Example setup 

```
    <param name="trajectory_type" type="int" value="1" />
    <param name="x" type="double" value="1" />
    <param name="y" type="double" value="0" />
    <param name="z" type="double" value="0.6"/>
    <param name="v" type="double" value="2" />
    <param name="move_target" type="bool" value="true"/>
    <rosparam param="waypoints_x">[0, 5, 5, 0, 0]</rosparam>
    <rosparam param="waypoints_y">[0, 0, 5, 5, 0]</rosparam>
    <rosparam param="waypoints_z">[0, 0, 0, 0, 0]</rosparam>
```

### Circle ##

Moves the agent in a circle. The only waypoint stated should be [0,0,0];

Example setup

```
    <param name="trajectory_type" type="int" value="2" />
    <param name="radius" type="double" value="3" />
    <param name="lambda" type="double" value = "-1"/>
    <param name="x" type="double" value="1" />
    <param name="y" type="double" value="0" />
    <param name="z" type="double" value="0.6"/>
    <param name="v" type="double" value="2" />
    <param name="move_target" type="bool" value="true"/>
    <rosparam param="waypoints_x">[0]</rosparam>
    <rosparam param="waypoints_y">[0]</rosparam>
    <rosparam param="waypoints_z">[0]</rosparam>
```

### Elipse ###

Moves the agent in an elipse. Use only two waypoints. 

```
    <param name="trajectory_type" type="int" value="3" />
    <param name="x" type="double" value="1" />
    <param name="y" type="double" value="0" />
    <param name="z" type="double" value="0.6"/>
    <param name="v" type="double" value="1" />
    <param name="move_target" type="bool" value="true"/>
    <rosparam param="waypoints_x">[0, 5]</rosparam>
    <rosparam param="waypoints_y">[0, 0]</rosparam>
    <rosparam param="waypoints_z">[0, 0]</rosparam>
```

## Use Plugin ##

The plugin has several parameters that will need to be set. 


| Name         | Description                                | Type  |
|--------------|--------------------------------------------|-------|
| kpPsi        | Heading proportional gain for PD control   | float |
| kpZ          | Altitude proportional gain for PD control  | float |
| kdPsid       | Heading differential gain for PD control   | foat  |
| kdZ          | Altitude differential gain for PD control  | float |
| maxVPsi      | Maximum velocity about z axis              | float |
| maxVZ        | Maximum linear velocity parallel to z axis | float |
| k_orbit      | Gain for orbital trajectory                | float |
| k_path       | Gain for linear trajectory                 | float |
| chi_infinity | Gain for linear trajectory                 | float |

Example setup

```
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
    </plugin>
```

## MovingTargets Service ##

The plugin contains a ROS server that allows the user to reset the agent at any time, or to tell the agent to stop or move. The service is defined below.

``` 
# Service message for moving targets

bool reset_target   # If true, the target will start over at its initial trajectory
bool move_target    # If true, the target will move

---

```
