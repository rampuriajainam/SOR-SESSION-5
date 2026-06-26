Session 5: Robotic Arm

# Table of Contents
1. [Introduction](#introduction)  
   1.1. [Download ROS package](#download-ros-package)  
2. [Building the robotic arm](#building-the-robotic-arm)  
   2.1. [Shoulder](#shoulder)  
   2.2. [Elbow](#elbow)  
   2.3. [Wrist](#wrist)  
   2.4. [Gripper](#gripper)  
   2.5. [Joint state publishing](#joint-state-publishing)  
3. [ROS Controller](#ros-controller)  
   3.1. [Joint trajectory control](#joint-trajectory-control)  
4. [Grabbing objects](#grabbing-objects)  
   4.1. [Using friction](#using-friction)  
   4.2. [Using detachable joints](#using-detachable-joints)  
5. [Detecting collision](#detecting-collision)  
6. [Adding an end effector](#adding-an-end-effector)  
7. [Simulating cameras](#simulating-cameras)  
   7.1. [Gripper camera](#gripper-camera)  
8. [MoveIt 2](#moveit-2)  
   8.1. [Changing the controller](#changing-the-controller)  
   8.2. [Setup assistant](#setup-assistant)  
   8.3. [Debugging](#debugging)  
   8.4. [Recap](#recap)  
   8.5. [Limitations](#limitations)  
   8.6. [Simplified startup](#simplified-startup)  

---

# Introduction

In this lesson we'll learn how to build a 4-axis robotic arm and make it move using ROS2 controllers and MoveIt 2.

## Download ROS package

To download the starter package, clone the following git repo with the `starter-branch` into your colcon workspace:

```bash
git clone -b starter-branch https://github.com/MOGI-ROS/Week-9-10-Simple-arm
```

---

# Building the robotic arm

The base of the robotic arm is already in the URDF file, but the colors in RViz and Gazebo don't match. Fix this first by including the materials in the URDF file:

```xml
  <!-- STEP 3 - RViz colors -->
  <xacro:include filename="$(find bme_ros2_simple_arm)/urdf/materials.xacro" />
```

Now we can proceed to adding the links of the robotic arm.

## Shoulder

The shoulder consists of two links: one for pan and one for lift. Add both to the URDF file:

```xml
  <!-- STEP 4 - Shoulder -->
  <joint name="shoulder_pan_joint" type="revolute">
    <limit lower="-3.14" upper="3.14" effort="330.0" velocity="3.14"/>
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
    <origin xyz="0.0 0.0 0.05" rpy="0.0 0.0 0.0"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Shoulder link -->
  <link name="shoulder_link">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.0014" ixy="0.0" ixz="0.0"
               iyy="0.0014" iyz="0.0"
               izz="0.0025"
      />
    </inertial>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="orange"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>

  <!-- Shoulder lift joint -->
  <joint name="shoulder_lift_joint" type="revolute">
    <limit lower="-1.5708" upper="1.5708" effort="330.0" velocity="3.14"/>
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 0.0 0.025" rpy="0.0 0.0 0.0"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Upper arm link -->
  <link name="upper_arm_link">
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.0012" ixy="0.0" ixz="0.0"
               iyy="0.0012" iyz="0.0"
               izz="0.0004"
      />
    </inertial>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
      <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
      <material name="orange"/>
      <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>
```

Rebuild the workspace and try it:

```bash
ros2 launch bme_ros2_simple_arm spawn_robot.launch.py
```

## Elbow

Add the elbow joint connecting the upper arm and forearm:

```xml
  <!-- STEP 5 - Elbow -->
  <joint name="elbow_joint" type="revolute">
    <limit lower="-2.3562" upper="2.3562" effort="150.0" velocity="3.14"/>
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Forearm link -->
  <link name="forearm_link">
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0.0 0.0 0.125" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.0011" ixy="0.0" ixz="0.0"
               iyy="0.0011" iyz="0.0"
               izz="0.0004"
      />
    </inertial>
    <collision>
      <geometry>
        <cylinder radius="0.025" length="0.25"/>
      </geometry>
      <origin xyz="0.0 0.0 0.125" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <cylinder radius="0.025" length="0.25"/>
      </geometry>
      <material name="orange"/>
      <origin xyz="0.0 0.0 0.125" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>
```

Rebuild the workspace and try it:

```bash
ros2 launch bme_ros2_simple_arm spawn_robot.launch.py
```

## Wrist

Add the wrist:

```xml
  <!-- STEP 6 - Wrist -->
  <joint name="wrist_joint" type="revolute">
    <limit lower="-2.3562" upper="2.3562" effort="54.0" velocity="3.14"/>
    <parent link="forearm_link"/>
    <child link="wrist_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 0.0 0.25" rpy="0.0 0.0 0.0"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Wrist link -->
  <link name="wrist_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0.0 0.0 0.05" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.00009" ixy="0.0" ixz="0.0"
               iyy="0.00009" iyz="0.0"
               izz="0.00002"
      />
    </inertial>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.1"/>
      </geometry>
      <origin xyz="0.0 0.0 0.05" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.1"/>
      </geometry>
      <material name="orange"/>
      <origin xyz="0.0 0.0 0.05" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>
```

Rebuild the workspace and try it:

```bash
ros2 launch bme_ros2_simple_arm spawn_robot.launch.py
```

## Gripper

Add the gripper, which consists of a base and two prismatic finger joints. Friction parameters are also improved for the fingers:

```xml
  <!-- STEP 7 - Gripper -->
  <joint name="gripper_base_joint" type="fixed">
    <parent link="wrist_link"/>
    <child link="gripper_base"/>
    <origin xyz="0.0 0 0.105" rpy="0.0 0 0"/> 
  </joint>

  <!-- Gripper base link -->
  <link name="gripper_base">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.00009" ixy="0.0" ixz="0.0"
               iyy="0.00009" iyz="0.0"
               izz="0.00002"
      />
    </inertial>
    <collision>
      <geometry>
        <box size=".05 .1 .01"/>
      </geometry>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <box size=".05 .1 .01"/>
      </geometry>
      <material name="grey"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>

  <!-- Left finger joint -->
  <joint name="left_finger_joint" type="prismatic">
    <limit lower="0" upper="0.04" effort="100.0" velocity="4.0"/>
    <parent link="gripper_base"/>
    <child link="left_finger"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 0.01 0.045" />
  </joint>

  <!-- Left finger link -->
  <link name="left_finger">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.00009" ixy="0.0" ixz="0.0"
               iyy="0.00009" iyz="0.0"
               izz="0.00002"
      />
    </inertial>
    <collision>
      <geometry>
        <box size=".04 .01 .08"/>
      </geometry>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <box size=".04 .01 .08"/>
      </geometry>
      <material name="blue"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>

  <gazebo reference="left_finger">
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <mu1>15</mu1>
    <mu2>15</mu2>
    <fdir1>1 0 0</fdir1>
    <maxVel>1.0</maxVel>
    <minDepth>0.002</minDepth>
  </gazebo>

  <!-- Right finger joint -->
  <joint name="right_finger_joint" type="prismatic">
    <limit lower="0" upper="0.04" effort="100.0" velocity="4.0"/>
    <parent link="gripper_base"/>
    <child link="right_finger"/>
    <axis xyz="0 -1 0"/>
    <origin xyz="0.0 -0.01 0.045" />
  </joint>

  <!-- Right finger link -->
  <link name="right_finger">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.00009" ixy="0.0" ixz="0.0"
               iyy="0.00009" iyz="0.0"
               izz="0.00002"
      />
    </inertial>
    <collision>
      <geometry>
        <box size=".04 .01 .08"/>
      </geometry>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <box size=".04 .01 .08"/>
      </geometry>
      <material name="blue"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>

  <gazebo reference="right_finger">
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <mu1>15</mu1>
    <mu2>15</mu2>
    <fdir1>1 0 0</fdir1>
    <maxVel>1.0</maxVel>
    <minDepth>0.002</minDepth>
  </gazebo>
```

Rebuild the workspace and try it:

```bash
ros2 launch bme_ros2_simple_arm spawn_robot.launch.py
```

## Joint state publishing

The `joint_state_publisher_gui` in RViz has no impact on the simulation. To move the arm in simulation we need to:

1. Turn off `joint_state_publisher_gui` in `spawn_robot.launch.py`.
2. Forward `joint_states` from Gazebo to ROS via `gz_bridge.yaml`:

```yaml
- ros_topic_name: "joint_states"
  gz_topic_name: "joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: "GZ_TO_ROS"
```

3. Add a `mogi_arm.gazebo` file in the `urdf` folder with a joint state publisher plugin:

```xml
<robot>
  <gazebo>
    <plugin
        filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
        <topic>joint_states</topic>
        <joint_name>shoulder_pan_joint</joint_name>
        <joint_name>shoulder_lift_joint</joint_name>
        <joint_name>elbow_joint</joint_name>
        <joint_name>wrist_joint</joint_name>
        <joint_name>left_finger_joint</joint_name>
        <joint_name>right_finger_joint</joint_name>
    </plugin>
  </gazebo>
</robot>
```

4. Include it in the URDF file:

```xml
  <!-- STEP 8 - Gazebo plugin -->
  <xacro:include filename="$(find bme_ros2_simple_arm)/urdf/mogi_arm.gazebo" />
```

Before moving to the next chapter, install the required packages:

```bash
sudo apt install ros-jazzy-controller-manager
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-joint-trajectory-controller
sudo apt install ros-jazzy-rqt-joint-trajectory-controller
```

---

# ROS Controller

Joint angles alone don't simulate actuators. `ROS2 control` provides controllers for each joint. Add it to the URDF file:

```xml
  <!-- STEP 9 - ROS2 control -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    <joint name="shoulder_pan_joint">
      <command_interface name="position">
        <param name="min">-2</param>
        <param name="max">2</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="shoulder_lift_joint">
      <command_interface name="position">
        <param name="min">-2</param>
        <param name="max">2</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="elbow_joint">
      <command_interface name="position">
        <param name="min">-2</param>
        <param name="max">2</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="wrist_joint">
      <command_interface name="position">
        <param name="min">-2</param>
        <param name="max">2</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="left_finger_joint">
      <command_interface name="position">
        <param name="min">-2</param>
        <param name="max">2</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="right_finger_joint">
      <command_interface name="position">
        <param name="min">-2</param>
        <param name="max">2</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
```

Also add the `ROS2 control` plugin to `mogi_arm.gazebo`:

```xml
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find bme_ros2_simple_arm)/config/controller_position.yaml</parameters>
    </plugin>
  </gazebo>
```

The controller `yaml` config is already in the package:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_joint
      - left_finger_joint
      - right_finger_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

Add the controller spawner to the launch file:

```python
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--param-file',
            robot_controllers,
            ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

## Joint trajectory control

Start the simulation and in another terminal launch the `joint_trajectory_controller` GUI:

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

Unlike `joint_state_publisher`, this sends real motion commands to the (real or simulated) joint controllers.

We can also add the `joint_state_broadcaster` to the launch file now:

```python
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

And remove the `joint_states` forwarding entry from `gz_bridge.yaml` since it's now handled by the broadcaster:

```yaml
- ros_topic_name: "joint_states"
  gz_topic_name: "joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: "GZ_TO_ROS"
```

After rebuilding, `joint_states` will be published by `joint_state_broadcaster`.

---

# Grabbing objects

There are two ways to interact with objects in the simulation: using friction and physics, or attaching/detaching objects with fake fixed joints.

## Using friction

Start the simulation:

```bash
ros2 launch bme_ros2_simple_arm spawn_robot.launch.py
```

In another terminal start the joint trajectory controller:

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

Adjust the arm angles to grab any of the objects. Grabbing with friction works well but requires Gazebo to simulate physics on all objects. In complex environments this can be expensive. Incorrect inertia matrices can also cause unstable objects that consume significant CPU time.

## Using detachable joints

An alternative is creating fixed joints between the object and the arm on demand. Add the `gazebo-detachable-joint-system` plugin to `mogi_arm.gazebo`:

```xml
  <gazebo>
    <plugin filename="ignition-gazebo-detachable-joint-system" name="ignition::gazebo::systems::DetachableJoint">
      <parent_link>left_finger</parent_link>
      <child_model>green_cylinder</child_model>
      <child_link>link</child_link>
      <detach_topic>/green/detach</detach_topic>
      <attach_topic>/green/attach</attach_topic>
      <output_topic>/green/state</output_topic>
    </plugin>
  </gazebo>
```

Forward the attach/detach topics in `gz_bridge.yaml`:

```yaml
- ros_topic_name: "/green/detach"
  gz_topic_name: "/green/detach"
  ros_type_name: "std_msgs/msg/Empty"
  gz_type_name: "gz.msgs.Empty"
  direction: "ROS_TO_GZ"

- ros_topic_name: "/green/attach"
  gz_topic_name: "/green/attach"
  ros_type_name: "std_msgs/msg/Empty"
  gz_type_name: "gz.msgs.Empty"
  direction: "ROS_TO_GZ"

- ros_topic_name: "/green/state"
  gz_topic_name: "/green/state"
  ros_type_name: "std_msgs/msg/String"
  gz_type_name: "gz.msgs.StringMsg"
  direction: "GZ_TO_ROS"
```

The detachable joint system starts with objects attached by default. Publish an empty message to `/green/detach` to detach, and to `/green/attach` to re-attach.

> A custom node that detaches all objects at startup is a cleaner long-term solution.

---

# Detecting collision

When multiple detachable objects are present, a contact sensor on the gripper finger can identify which object is being touched, enabling dynamic attachment of the correct object.

Add a contact sensor to the left finger in `mogi_arm.gazebo`:

```xml
  <gazebo reference="left_finger">
    <sensor name='sensor_contact' type='contact'>
      <contact>
        <collision>left_finger_collision</collision>
        <topic>/contact_left_finger</topic>
      </contact>
      <always_on>1</always_on>
      <update_rate>100</update_rate>
    </sensor>
  </gazebo>
```

Forward the contact topic in `gz_bridge.yaml`:

```yaml
- ros_topic_name: "/contact_left_finger"
  gz_topic_name: "/contact_left_finger"
  ros_type_name: "ros_gz_interfaces/msg/Contacts"
  gz_type_name: "gz.msgs.Contacts"
  direction: "GZ_TO_ROS"
```

Rebuild and start the simulation. Touch an object with the left finger and monitor the `/contact_left_finger` topic in `rqt` to see the contact information including the child object's name.

---

# Adding an end effector

Adding a visual marker for the tool center point (TCP) makes it easier to track the end effector pose in 3D space. Add a small red cube with no collision geometry to `mogi_arm.xacro`:

```xml
  <!-- STEP 10 - End effector -->
  <joint name="end_effector_joint" type="fixed">
    <origin xyz="0.0 0.0 0.175" rpy="0 0 0"/>
    <parent link="wrist_link"/>
    <child link="end_effector_link"/>
  </joint>

  <!-- End effector link -->
  <link name="end_effector_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.01 0.01 0.01" />
      </geometry>
      <material name="red"/>
     </visual>

    <inertial>
      <origin xyz="0 0 0" />
      <mass value="1.0e-03" />
      <inertia ixx="1.0e-03" ixy="0.0" ixz="0.0"
               iyy="1.0e-03" iyz="0.0"
               izz="1.0e-03" />
    </inertial>
  </link>
```

---

# Simulating cameras

## Gripper camera

Add a camera to the gripper. Start with the URDF:

```xml
  <!-- STEP 11 - Gripper camera -->
  <joint type="fixed" name="gripper_camera_joint">
    <origin xyz="0.0 0.0 0.0" rpy="0 -1.5707 0"/>
    <child link="gripper_camera_link"/>
    <parent link="gripper_base"/>
  </joint>

  <link name='gripper_camera_link'>
    <pose>0 0 0 0 0 0</pose>
    <inertial>
      <mass value="1.0e-03"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".01 .01 .01"/>
      </geometry>
      <material name="red"/>
    </visual>
  </link>

  <joint type="fixed" name="gripper_camera_optical_joint">
    <origin xyz="0 0 0" rpy="-1.5707 0 -1.5707"/>
    <child link="gripper_camera_link_optical"/>
    <parent link="gripper_camera_link"/>
  </joint>

  <link name="gripper_camera_link_optical">
  </link>
```

Add the Gazebo camera plugin to `mogi_arm.gazebo`:

```xml
  <gazebo reference="gripper_camera_link">
    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>15</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
        <optical_frame_id>gripper_camera_link_optical</optical_frame_id>
        <camera_info_topic>gripper_camera/camera_info</camera_info_topic>
      </camera>
      <always_on>1</always_on>
      <update_rate>20</update_rate>
      <visualize>true</visualize>
      <topic>gripper_camera/image</topic>
    </sensor>
  </gazebo>
```

Forward `camera_info` in `gz_bridge.yaml`:

```yaml
- ros_topic_name: "gripper_camera/camera_info"
  gz_topic_name: "gripper_camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: "GZ_TO_ROS"
```

Add the image bridge and relay nodes to the launch file:

```python
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/gripper_camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'gripper_camera.image.compressed.jpeg_quality': 75},
        ],
    )

    relay_gripper_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['gripper_camera/camera_info', 'gripper_camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

---

# MoveIt 2

Writing custom inverse kinematics works for simple robots but becomes complex with more joints, constrained workspaces, or collision requirements. MoveIt 2 is a comprehensive ROS 2 framework for robot motion planning providing:

- Inverse kinematics
- Path planning
- Collision checking
- Trajectory execution
- Grasp planning

Install MoveIt 2:

```bash
sudo apt install ros-jazzy-moveit
```

## Changing the controller

MoveIt requires the gripper fingers to be in a separate controller. Update `controller_position.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

gripper_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - left_finger_joint
      - right_finger_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

Update the controller spawner in the launch file to load both controllers:

```python
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            'gripper_controller',
            '--param-file',
            robot_controllers,
            ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )
```

## Setup assistant

Generate the MoveIt configuration package using the setup assistant:

```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```

Follow these steps in the GUI:

1. Press **Create New MoveIt Configuration Package**, then browse and load `mogi_arm.xacro`. The robot is visualized on the right when loaded successfully.

2. Go to **Self-Collision** and press **Generate Collision Matrix**.

3. Go to **Planning Groups** and add a group for the arm. Set the **Kinematic Solver**, then add a kinematic chain with `base_link` as the start and `end_effector_link` as the end.

4. Add a second group for the gripper (no kinematic solver needed). Use **Add Joints** and select `left_finger_joint` and `right_finger_joint` only.

5. Go to **Robot Poses** and add default poses such as a home position and open/closed gripper states.

6. Go to **End Effectors** and add the `end_effector_link`.

7. Go to **ROS 2 Controllers** and press **Auto Add JointTrajectoryController**.

8. Go to **MoveIt Controllers** and press **Auto Add JointTrajectoryController**.

9. Fill in author information, then browse to the parent folder of `bme_ros2_simple_arm` and generate the new package with the name `bme_ros2_simple_arm_moveit_config`. Press **Generate Package**.

You can ignore the warning about missing virtual joints. Exit the setup assistant when done.

## Debugging

Rebuild the workspace, source `install/setup.bash`, and start the simulation:

```bash
ros2 launch bme_ros2_simple_arm spawn_robot.launch.py
```

In another terminal start MoveIt's `move_group`:

```bash
ros2 launch bme_ros2_simple_arm_moveit_config move_group.launch.py
```

### Fix: integer joint limits

If you see this error:

```
parameter 'robot_description_planning.joint_limits.left_finger_joint.max_velocity' has invalid type: expected [double] got [integer]
```

Edit `bme_ros2_simple_arm_moveit_config/config/joint_limits.yaml` and change every `max_velocity` and `max_acceleration` to a double for all joints:

```yaml
  left_finger_joint:
    has_velocity_limits: true
    max_velocity: 4.0
    has_acceleration_limits: false
    max_acceleration: 0.0
```

### Fix: missing acceleration limits

After the previous fix you'll see:

```
No acceleration limit was defined for joint shoulder_pan_joint!
```

Add acceleration limits for every joint in the same `joint_limits.yaml`:

```yaml
    has_acceleration_limits: true
    max_acceleration: 3.14
```

### Fix: controller not found

After the above fixes you may see:

```
Unable to identify any set of controllers that can actuate the specified joints
```

Edit `bme_ros2_simple_arm_moveit_config/config/moveit_controllers.yaml` and add the missing `action_ns` and `default: true` fields:

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller

  arm_controller:
    type: FollowJointTrajectory
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_joint
    action_ns: follow_joint_trajectory
    default: true
  gripper_controller:
    type: FollowJointTrajectory
    joints:
      - left_finger_joint
      - right_finger_joint
    action_ns: follow_joint_trajectory
    default: true
```

### Fix: simulation time

After all the above fixes, planning still fails with:

```
Didn't receive robot state (joint angles) with recent timestamp within 1.000000 seconds.
Check clock synchronization if you are running ROS across multiple machines!
```

MoveIt is not using simulation time. Set the parameter while MoveIt is running:

```bash
ros2 param set /move_group use_sim_time true
```

Then start RViz:

```bash
ros2 launch bme_ros2_simple_arm_moveit_config moveit_rviz.launch.py
```

Use the interactive marker to set a goal pose and press **Plan & Execute**. It should now plan and execute correctly in both RViz and Gazebo.

## Recap

Complete startup sequence for MoveIt:

**Terminal 1** — start the simulation without RViz:
```bash
ros2 launch bme_ros2_simple_arm spawn_robot.launch.py rviz:=False
```

**Terminal 2** — start the MoveIt `move_group` backend:
```bash
ros2 launch bme_ros2_simple_arm_moveit_config move_group.launch.py
```

**Terminal 3** — start RViz from the MoveIt package:
```bash
ros2 launch bme_ros2_simple_arm_moveit_config moveit_rviz.launch.py
```

**Terminal 4** — set simulation time:
```bash
ros2 param set /move_group use_sim_time true
```
