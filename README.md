# java-notes
## motion planning平台搭建（一）
概述：这部分主要关于模型建立，MoveIt config文件，给模型添加控制器等。

---
### 1 模型建立
这次基于abb1200_7_700，负重7KG，工作空间700mm。模型下载地址见[ros官方给的github地址](https://github.com/ros-industrial/abb_experimental)。这里的模型直接拿来用好像会出问题，下面会根据遇到的问题依次解决。

因为是只用urdf模型即可，所以直接进入abb_irb1200_support->urdf，可以看到有两种规格的机器人模型，因为实验室的规格是7_700,所以选择irb1200_7_70_macro.xacro文件。进入文件基本就是定义的link、joint标签等。但是因为我们下载的文件包中没有abb_resources资源包（我在其他地方也没有找到），所以需要将第一句话屏蔽了。

```
<!--<xacro:include filename="$(find abb_resources)/urdf/common_materials.xacro"/> -->
```
同理，与该文件相关的还有给每个link定义颜色信息的字段

```
<!--<xacro:material_abb_yellow />-->
```
这时，整个.xacro文件运行就会正常了，虽然模型整体没有颜色，但是不影响我们仿真，并且，实验室的机器人也正好是白色。

这时，我的选择是把.xacro文件转换成.urdf文件。
在urdf目录下打开终端，输入：

```
rosrun xacro xacro.py irb1200_7_70.xacro > irb1200_7_70.urdf
```
irb1200_7_70.xacro是.xacro文件名，irb1200_7_70.urdf是生成的urdf文件名

在终端中接着输入：

```
urdf_to_graphiz irb1200_7_70.urdf
```
生成一个pdf文件，可以通过这个pdf文件查看模型的整体结构。

注意：为了能让模型在world中固定，我在irb1200_7_70.urdf最后加上了以下字段：

```
  <link name="world"/>
  <joint name="fixed" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
  </joint>
```
同时，为了能让模型在gazebo中显示（可能后期会用得到），我给每个link加上了<inertial>标签（这里可能数值不对，但是问题不大）：

```
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
```
最后，为了能控制机器人运动（后期可能在需要在gazebo仿真），必须给机器人的joint加上制动器。如下依次给每个joint加上制动器：

```
    <!-- joint_1 transmission -->
  <transmission name="tran1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint_1">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor1">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
```
这是joint_1相关，其他关节类似。

这时，irb1200_7_70的urdf模型就建立好了。

---

### 2 MoveIt config文件
这一步根据moveit助手生成Moveit config文件。

##### （1）首先，启动moveit助手
在终端中输入：

```
 roslaunch moveit_setup_assistant setup_assistant.launch
```
在界面导入我们的urdf文件，然后可以在右边窗口看到irb1200的模型。

注：旁边有个edit existing Moveit Configuration Package,当我们的模型变化了之后，可以通过这个来导入新的模型，覆盖原有功能包中的文件内容。

##### （2）自碰撞矩阵
点击左边第二项"self-collisions"，这里采用默认配置，默认采样点是10000个，点击生成，即可生成碰撞矩阵。
##### （3）虚拟关节
虚拟关节主要是用来描述机器人在world坐标系下的位置。如果机器人是移动的，则虚拟关节可以与移动基座关联，但这里我们的机械臂是固定不动的，所以无需设置虚拟关节。
##### （4）创建规划组
规划组，顾名思义，就是用来规划的一组link和joint，我们这里是规划整个机器人，没有其他额外部分，所以就把所有的link和joint设置成一个规划组。那么moveit在之后的运动规划过程中会针对我们定义的规划组进行运动学解析。

点击"Add Group"，设定规划组名字等参数，这个名字之后在编程中用得到：
- Group Name : irb1200
- Kinematic Solver : kdl_kinematics_plugin/KDLKinematicsPlugin
- Kin.Search Resolution : 0.005
- Kin.Search Timeout(sec) : 0.005
- Kin.Solver Attempts : 3

然后点击Add joints（这里点add kin.chain也可以，原理是一样的），选择一共有9个joint,除了base_link-base，其他全部选择。可以通过点击joint的名字查看joint的位置，判断是否存在问题。最后点击save。

最后可以看到当前组是irb1200,joints下面有8个joint，头和尾都是fixed，其他是revolute。
##### （5）定义机器人位姿
这部分可以给固定的位姿命名，然后程序中通过名字来调用。例如定义home点。
点击add pose ，输入home，各个关节均为0值。点击save。
也可以根据需要再自己添加几个位姿。
##### （6）配置EE
因为这里的机器人没有EE，所以就跳过。
##### （7）配置无用关节
机器人上的某些关节可能在规划、控制过程中使用不到，可以先声明出来。我们的机器人没有类似的joint，这一步跳过。
##### （8）设置作者信息
根据自身情况填写。
##### （9）生成配置文件
Setup Assistant会将之前的所有配置打包成一个ROS功能包，一般这个包的命名为RobotName_moveit_config，所以我们命名位irb1200_moveit_config。直接点击Browse，定位到我们的ws目录src，例如我最后的路径名为：

```
/home/xbbleon/catkin_ws/src/irb1200_moveit_config
```
点击Generate Package，会弹出一个窗口提示我们有的配置没做，点击OK，然后会看到生成成功。点击退出。

### 3 测试模型
首先在ws中编译生成的功能包。
编译完成后输入：

```
roslaunch irb1200_moveit_config demo.launch 

```
然后可以在rviz中用motion planning插件来进行轨迹规划。

### 4 添加关节控制器
MoveIt!默认生成的demo中使用的控制器功能有限，我们需要使用其他控制器插件实现驱动机器人模型的功能。一种方法是用gazebo和moveit，使用ros_control作为工具，这个会用到gazebo相对来说比较麻烦。另一种是添加ArbotiX关节控制器，ArbotiX功能包中提供了JointTrajectoryActionControllers插件，可以用来驱动真实机器人，实现旋转运动。

先梳理下配置流程：

（1）在加载机器人模型的时候，ArbotiX控制器需要以节点的形式加载。
（2）moveit方面需要配置控制器来和上面定义的控制器通信
（3）如何通信，也就是把上面两个结合起来。

###### 首先是第一步，添加ArbotiX控制器。
（这部分控制器后期可以用gazebo的控制器替代。）
之前我们先创建一个功能包，用来放置我们接下来要写的launch文件和之后的程序节点。在终端中依次输入以下指令：

```
cd ~/catkin_ws/src
catkin_create_pkg irb1200_control std_msgs rospy roscpp
```
上面catkin_ws是我的ws。

然后在irb1200_control文件夹下创建config,launch文件夹。

在config目录下，新建文件irb1200.yaml,具体内容如下：

```
joints: {
    joint_1: {id: 1, range: 360,max_angle: 169.9, min_angle: -169.9, max_speed: 288},
    joint_2: {id: 2, range: 360,max_angle: 134.9, min_angle: -99.9, max_speed: 240},
    joint_3: {id: 3, range: 400,max_angle: 69.9, min_angle: -200, max_speed: 300},
    joint_4: {id: 4, range: 540,max_angle: 269.9, min_angle: -269.9, max_speed: 400},
    joint_5: {id: 5, range: 360,max_angle: 130.0, min_angle: -130.0, max_speed: 400},
    joint_6: {id: 6, range: 720,max_angle: 360, min_angle: -360, max_speed: 600},
}
controllers: {
    irb1200_controller: {type: follow_controller, joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], action_name: irb1200_controller/follow_joint_trajectory, onboard: False }
}
```
里面的数值根据urdf文件相应计算，如与真实数据有很大出入，再修改。[注意这个range属性！！！]

###### 接着是第二步，配置moveit相关的控制器
回到irb1200_moveit_config文件夹，进入config目录，新建文件controllers.yaml，并写入以下内容：

```
controller_list:
  - name: irb1200_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
```
然后在launch目录中去配置并加载这个文件。

打开abb_irb1200_7_70_moveit_controller_manger.launch.xml,这个文件通过trajectory_execution.launch.xml启动，而trajectory_execution.launch.xml由move_group.launch加载。

文件abb_irb1200_7_70_moveit_controller_manger.launch.xml默认是空文件，我们需要添加以下内容：
```
<launch>
    <!-- Set the param that trajectory_execution_manager needs to find the controller plugin -->
    <arg name="moveit_controller_manager" default="moveit_simple_controller_manager/MoveItSimpleControllerManager" />
    <param name="moveit_controller_manager" value="$(arg moveit_controller_manager)"/>

    <!-- load controller_list -->
    <arg name="use_controller_manager" default="true" />
    <param name="use_controller_manager" value="$(arg use_controller_manager)" />

    <!-- Load joint controller configurations from YAML file to parameter server -->
    <rosparam file="$(find irb1200_moveit_config)/config/controllers.yaml"/>
</launch>
```
这时两端已经配置好了，开始配置联通的工具。

###### 第三步，配置结合的接口
其实，我们在编写.yaml文件的时候就已经将两者提前配对了，比如控制器的名字都叫ibr1200_controller,action都是follow_joint_trajectory，所以这里只用写launch文件即可。

进入ibr1200_control的launch目录下，新建motion_planning.launch文件，并输入以下内容：

```
<?xml version="1.0"?>
<launch>
    <!--启动机器人模型-->
    <!--<include file="$(find irb1200_moveit_config)/launch/demo.launch"/>-->
    <!-- 不使用仿真时间 -->
    <param name="/use_sim_time" value="false" />

    <!-- 启动arbotix driver-->
    <arg name="sim" default="true" />

    <node name="arbotix" pkg="arbotix_python" type="arbotix_driver" output="screen">
        <rosparam file="$(find irb1200_control)/config/irb1200.yaml" command="load" />
        <param name="sim" value="true"/>
    </node>

    <node pkg="robot_state_publisher" type="robot_state_publisher" name="rob_st_pub" />
    <include file="$(find irb1200_moveit_config)/launch/move_group.launch"/>
    <!-- 启动rviz -->
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find irb1200_control)/urdf.rviz" required="true" />
    <param name="robot_description" textfile="$(find irb1200_control)/urdf/irb1200_7_70.urdf"/>
</launch>
```
在ibr1200_control新建urdf文件夹，将之前生成的irb1200_7_70.urdf放入该文件夹下。在ibr1200_control文件夹下拷贝urdf.rviz文件。

### 5 moveit测试
在终端中输入：

```
roslaunch irb1200_control motion_planning.launch
```
可以把机器人模型和move_group节点加载到rviz中显示。

这时，我们再另外编写一个节点，通过编程的方式来使用moveit插件，进行运动规划。

实现motion planning的节点方式有python 和c++两种，这里先用python，后面教程的代码包括碰撞避免，轨迹插值等都会使用c ++,两种原理一样。
在ibr1200_control文件夹下新建scripts文件夹，并新建文件moveit_fk_demo.py,并写入一些内容：

```
#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander

class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('irb1200')
       
        
        # 设置机械臂和夹爪的允许误差值
        arm.set_goal_joint_tolerance(0.001)
        
        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)
       
         
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions = [-0.0867, -1.274, 0.02832, 0.0820, -1.273, -0.003]
        arm.set_joint_value_target(joint_positions)
                 
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
```
在package.xml中添加以下依赖，有的后面会用到就一并添加了，这里直接将原来的依赖覆盖即可：

```
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>	
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>message_generation</build_export_depend>
  <build_depend>moveit_core</build_depend>
  <build_depend>moveit_commander</build_depend>
  <build_depend>moveit_ros_planning_interface</build_depend>
  <build_depend>moveit_ros_perception</build_depend>
  <build_depend>interactive_markers</build_depend>
  <build_depend>geometric_shapes</build_depend>
  <build_depend>pcl_ros</build_depend>
  <build_depend>pcl_conversions</build_depend>
  <build_depend>rosbag</build_depend>
  <build_depend>actionlib</build_depend>
  <build_depend>trajectory_msgs</build_depend>

  <exec_depend>actionlib</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>pluginlib</exec_depend>
  <exec_depend>moveit_core</exec_depend>
  <exec_depend>moveit_fake_controller_manager</exec_depend>
  <exec_depend>moveit_ros_planning_interface</exec_depend>
  <exec_depend>moveit_ros_perception</exec_depend>
  <exec_depend>interactive_markers</exec_depend>
  <exec_depend>joy</exec_depend>
  <exec_depend>pcl_ros</exec_depend>
  <exec_depend>pcl_conversions</exec_depend>
  <exec_depend>rosbag</exec_depend>
  <exec_depend>moveit_commander</exec_depend>
  <exec_depend>trajectory_msgs</exec_depend>
```
修改CMakeLists.xml文件，直接覆盖原来的即可：

```
cmake_minimum_required(VERSION 2.8.3)
project(irb1200_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  moveit_commander
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

```
最后，将moveit_fk_demo.py变成可执行文件，在scripts文件夹下，打开终端，输入：

```
chmod +x moveit_fk_demo.py
```
最后编译一下ws。

编译完成后查看一下效果，在两个终端里分别输入：

```
roslaunch irb1200_control motion_planning.launch
rosrun irb1200_control moveit_fk_demo.py（可以多次输入，会来回运动）
```
可以看到机器人的运动效果。下个教程会讲如何用c++来实现机器人路径规划，以及往场景中添加八叉树图并进行碰撞避免的路径规划。

