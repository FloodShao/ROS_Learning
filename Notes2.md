# 1. 创建catkin_package
使用 catkin_create_pkg package_name {dependencies} 来创建一个package
这条命令会在当前文件夹下创建CMakelists.txt 和 package.xml 两个文件，其中package_name是相同的

在该项目中{dependencies} 为roscpp和sensor_msgs，建议在使用catkin_create_pkg的时候就将这两个dependencies加上，

**查看CMakeLists.txt**

* cmake_minimum_required
* project(*project_name*)
* find_package(catkin REQUIRED COMPONENTS ... dependencies)
* catkin_package(*INCLUDE_DIRS include*, *CATKIN_DEPENDS roscpp sensor_msgs* ...)
* include_directories( include ${catkin_INCLUDE_DIRS})
* add_executable(\${PROJECT_NAME} src/\${PROJECT_NAME}_node.cpp src/TestControl.cpp}) //添加可执行文件的编译 
* target_link_libraries(\${PROJECT_NAME} \${catkin_LIBRARIES}) //这个一定不要忘掉了

**package.xml**
这个主要检查几个dependencies是否<build></build>

# 2. package的基本组成(按重要顺序)

* src 放置执行文件
* include 放置头文件 （定义class）
* config 放置参数文件 param.yaml
* launch 放置launch文件
* 未完待续

src中，主要放置node.cpp可执行文件，和class的定义文件。其中main函数在node.cpp中

# 3. 编译package
没有采用eclipse的编译方式，（可能在ide环境中进行编写代码会减少错误等）
在你建立的catkin_ws文档中，使用
ln -s source_dir target_dir
进行连接
在catkin_ws中进行
catkin build pakcage_name
来进行单个包的编译，根据编译后的报错信息来进行调试

**在每一次编译一个包之后都要对devel/setup.bash进行source更新**

# 4. 关于加载config.yaml中的parameter
需要使用
ros::NodeHandle.getParam( string parameter_name_in_config, string parameter_name_defined_in_your_class)
需要对应parameter的名字，一次只能加载一个，所以需要在node class中添加读取config参数的定义函数

参考ros_package_template中的readParameters()

一般会将该函数定义为bool返回类型，来检查参数是否加载正确，如果不正确需要ROS_ERROR

# 5. 在launch文件中添加node
**roslaunch 不保证节点开始的顺序，因为没有办法从外部知道节点何时完全被初始化，因此所有的节点必须是稳健的，以便以任何顺序进行启动**
```
<node pkg="test" name="test" type="test" output="screen" launch-prefix="gnome-terminal --command">
    <rosparam command="load" file="$(find test)/config/config.yaml"/>
</node>
```
**node标签**
* pkg="mypackage" 节点包
* type="nodetype" 节点类型，必须具有一个相同名称的节点类型
* name="nodename" 节点基名称，注意name不能包含命名空间
* args="arg1 arg2 agr3" 传递参数到节点
* machine="machine name" 在指定机器上启动节点
* output=”log|screen“ 只有两种选择，在log文件中或者在屏幕上的terminal显示
* launch-prefix=”prefix argument“ 用于添加到节点中的启动命令、参数


**元素**
可以在<node>标签下添加一下XML标签：
<env>为节点设置的环境变量
<remap>为节点设置的重新映射参数
<rosparam>将rosparam文件加载到此节点的~/local命令空间中
<param>在节点的~/local命名空间中设置一个参数

# 6. launch file 中添加其他launch文件，来同时激活多个node
```
<include file="$(find package_name)/launch/launch_file.launch">
    <arg name="argname" value="$(arg_defined_value)"/>
</include>
```

launch file 标签group，定义命名空间ns。
```
以下方法可以同时启动同一个node而不会产生冲突
<group ns="turtlesim1" >
    <node pkg="turtlesim" type="turtlesim_node" name="sim">
</group>
<group ns="turtlesim2" >
    <node pkg="turtlesim" type="turtlesim_node" name="sim">
</group>
```


# 7. 使用rviz
rviz是ROS官方的一款3D可视化工具，几乎我们需要用到的所有机器人相关数据都可以在rviz中展现。
使用rosrun rviz rviz启动ROS系统中的rviz平台
也可以在launch文件中添加rviz node来启动rviz平台
<node pkg="rviz" type="rviz" name="rviz">

rviz 主要包含以下几个部分：

* 3D视图区，中间显示区
* 工具栏，顶部工具栏
* 显示项列表，用于显示当前选择的显示插件，可以配置每个插件的属性，位于左侧
* 视角设置区，可以选择多种观测视角，位于右侧
* 时间显示区，显示当前的系统时间和ROS时间

**进行数据可视化的前提是要有数据**。假设需要可视化的数据以对应的消息类型发布，我们在rviz中使用相应的插件订阅该消息即可实现显示。
在显示项列表中的add中添加需要显示的数据。添加完成后，rviz左侧的显示项列表会出现已经添加的显示插件。注意”Topic“属性，这个是用来声明该显示插件所订阅的数据来源，如果订阅成功，在中间的显示区应该会出现可视化后的数据。（注意有时候是因为数据点size不够大所以看不见）
如果显示有问题，检查属性区域的”Status“状态，status有四种状态：OK、Warning、Error和Disabled，如果显示的状态不是OK，那么需要查看错误信息，并详细检查数据发布是否正常。



```
1. launch file 中，$() 而不是${}
```



