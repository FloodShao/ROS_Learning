# 1. TF Transformation System

* tool for keeping track of coordinate frames over time
* maintains relationship between coordinate frames in a tree structure buffered in time
* lets the user transform points, vector, etc. between coordinate frames at desired time
* implemented as publisher/subscriber model on the topics /tf and /tf_static

```
tf2_msgs/TFMessage.msg
可以通过
rosmsg show TFMessage 来查找相关msg定义
```

```
rosmsg 解析： rosmsg show geometry_msg/Twist
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z

其中linear和angular都是复合域，即本质上是向量的关系，这个msg总共有6个参数，每3个编为一个向量进行组织。
消息类型同样可以包含固定或可变长度的数组（常用[]来表示）
```

# 2. rqt 用户端口

* User interface based on Qt
* Custom interfaces can be setup
* Lots of plugins exit
* Simple to write own plugins

```
启动rqt：
rosrun rqt_gui rqt_gui  或
rqt
```

# 3.URDF unified robot description format

* defines an XML format for representing a robot model
* URDF generation can be scripted with XACRO
* the robot description is stored on the parameter server under */robot_description*

# 4. Simulation Descriptions
simulation description format (SDF)

* defines an XML format to describe: environments, objects, sensors, robots
* sdf is the standard format for gazebo
* gazebo converts a URDF to SDF automatically


# 5. ros:init()
这是ROS程序调用的第一个函数，用于对ROS程序的初始化。是在定义一个ROS node过程中进入main函数后第一个函数。
在创建ros::NodeHandle之前必须先对ROS程序进行初始化ros::init( argc, argv, string package_name, uint32_t options);

# 6. ROS 创建 Subscriber 和 Publisher
Subscriber和Publisher都是ros::NodeHandle下定义的类。

```
首先，定义
ros::NodeHandle nodeHandle_ = nodehandle; //从main函数中传入nodehandle

ros::Subscriber sub_;
ros::Publisher pub_;
注意现在只是定义了类，但并没有与nodeHandle_产生联系

sub_ = nodeHandle_.subscribe(topic, queue_size, &callback_function, this);
/*
* 其中 topic为std::string
* queue_size为int。queue_size取1到2，表示只对当前的状态感兴趣，因为数据较多就会溢出，
* 只能保留当前最新的状态；queue_size = 0表示无限队列，同样会影响稳定性
* 如果取值较大，则说明该topic的数据需要按顺序进行处理，不能舍弃。
* callback_function一般为void无返回值类型
*/

pub_ = nodeHandle_.advertise<message_type>(topic, queue_size);
/*
* 一定要注意message_type, 一定要符合include的message_type,
* topic要也是std::string 一般会从yaml文件中获取
* queue_size与上边一样，取queue_size = 0为无限队列，会影响稳定性
*/
```

Subscriber自动监听topic，在callback函数中进行后续操作。
Publisher需要进行pub_.publish(msgs) 才能对外广播。

# 7. 汽车坐标系
汽车的前进方向为x轴，对应侧倾角，滚转角 roll；
指向汽车左侧方向为y轴，对应俯仰角，pitch；
指向汽车正上方为z轴，对应偏航角，yaw；











