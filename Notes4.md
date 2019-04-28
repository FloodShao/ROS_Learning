# 1. ROS Services

* Request/Response communication between nodes is realized with services. (服务器发布服务，客户终端接受服务)
* service在*.srv文件中进行定义
    可以查询当前所有的服务 rosservice list
    查询服务类型 rosservice type /service_name
    呼叫一次服务 rosservice call /service_name args
    查看服务定义 ros
    
Request 和 Response 用三条横线分开
```
举例：nav_msgs/GetPlan.srv
geometry_msgs/PoseStamped start  //Request
geometry_msgs/PoseStamped goal
---
nav_mags/Path plan  //Response
```

在使用
rosservice call service_name "args"
时使用tab来进行args的填写

* 定义一个Service Server
ros::ServiceServer service = nodeHandle.advertiseService(service_name, callback_function);
当一个服务器接收到request的时候，callback_function进行调用，并将request作为参数；
并生成response同样作为参数返回argument，传址操作；
callback_function返回bool类型作为service是否调用成功；

* 定义一个Service Client
ros::ServiceClient client = nodeHandle.serviceClient<service_type>(service_name);
创建service request内容service.request, 并填写request内容；
呼叫service：client.call(service)。注意这时候service中的request已经填写了内容。
当client.call返回true的时候，response存储在service.response中

# ROS actions （actionlib）
Similar to service call, but provide possibility to：(1) cancel the task; (2) receive feedback on the progress

Best way to implement interfaces to time-extended, goal-oriented behaviors.

Similar in structure to services, action are defined in *.action files.

Internally, actions are implemented with a set of topics.

```
*.action

Goal
---
Result
---
Feedback
```

# ROS 几项功能的对比


|   | Parameters | Dynamic Reconfigure | Topics | Services | Actions |
| --- | --- | --- | --- | --- | --- |
| Description | Global constant parameters | Local, changeable parameters | Continuous data streams | Blocking call for processing a request | Non-blocking preemptable oriented task  |
|Application |Constant Settings | Tuning parameters | One-way continuous data flow | Short triggers or calculations |Task executions and robot actions|
|Examples| Topic name, camera settings, calibration data, robot setup| Controller parameters | Sensor data, robot sate| Trigger change, request state, compute quantity| Navigation, motion execution|


# ROS Time
一般情况下，ROS使用计算机系统时钟作为时钟源
对于仿真文件或者log文件回放，ROS会使用仿真时钟。

* rosparam set use_sim_time true //使用仿真时钟
* 或者向/clock 这个rostopic发布时间

ros::Time begin = ros::Time::now();
double secs = begin.toSec();

ros::Duration duration(0.5);

ros::Rate rate(10); //10Hz


# ROS Bag
A bag is a format for storing message data
它是一个二进制文件 *.bag
Suited for logging and recording datasets for future visualization and analysis

rosbag record -all //record all topics in a bag
rosbag record topic_1 topic_2 topic_3 //record given topics

stop recording with ctrl+c

show information about a bag:
rosbag info bag_name.bag

Read a bag and publish its contents:
rosbag play bag_name.bag

Playback options can be defined:
rosbag play --rate-0.5 bag_name.bag
[参数] 
--rate=factor //publish rate
--clock //publish the closk time
--loop //loop play back

# 作业笔记

1. 使用rqt_multiplot显示/odometry/filtered轨迹。但是rospack找不到这个package。查询显示没有安装

可以通过 sudo apt-get install ros-kinetic-rqt-multiplot 来进行安装






