# ROS
ROS = Robot Operating System
1. Plumbing
2. Tools
3. Capabilities
4. Ecosystem

Philosophy

1. Peer to peer communication
    Individual programs communicate over define API (ROS msgs, services, etc)
    
2. Distributed
3. Multi-Lingual
    C++ Python, Matlab, Java
4. Light-Weight
5. Free and Open

ROS master manages the msgs, start with **roscore**
ROS nodes, single-purpose, individually compiled, organized in pakages
    rosrun package_name
    rosnode list
ROS Topics, node communicate over topics

尽量使用catkin build pakage_name
注意使用catkin build需要安装python_catkin_tools这个包
Whenever you build a new package, update your environment
source devel/setup.bash

```
linux 建立链接
ln -s 源目录 目标目录
-s (symbolic) 建立软链接，并不会重新建立文件，只会建立一个同步链接
如果没有-s则会建立硬链接，重新建立文件，

注意无论是软链接还是硬链接，都会在两个文件目录中进行同步，修改一处就会修改另一处
```

ROS launch is a tool for launching multiple nodes (as well as parameters). Are written in XML as *.launch files
Start a launch file from a package: > roslaunch package_name file_name.launch
When launching, arguments can be set with >roslaunch launch_file.launch arg_name:=value


# ROSTopic
rostopic list 可以查看所有的rostopic
rosmsg show可以查看当前rosmsg的格式是什么
利用rostopic在终端进行命令控制：
rostopic pub [parm] [topic] [type] [args]
例如：在husky_simulation中对cmd_vel 的geometry_msgs/Twist进行控制 （-r 10）表示以10hz的频率进行publish
rostopic pub -r 10 /cmd_vel /geometry_msgs/Twist -- '[0.1, 0.0, 0.0]' '[0.0., 0.0, 0.1]'
千万注意参数中的空格，输入参数的格式

# 环境配置问题
1. catkin build 需要安装python_catkin_tools (apt 安装)
2. 进行catkin build ros_package_template 的时候发现如下错误：

ImportError: "from catkin_pkg.package import parse_package" failed: No module named catkin_pkg.package
Make sure that you have installed "catkin_pkg", it is up to date and on the PYTHONPATH.

这里提示找不到catkin_pkg。注意catkin_pkg有两种安装方式，回到用户根目录，使用pip进行安装。
pip install catkin_pkg

安装成功之后，编译成功

3. 如果觉得配置失败，可以使用catkin clean将生成的devel,build和logs目录删掉重新进行配置
4. 在进行gazebo husky仿真的时候，出现no module called rospkg found.
这个仍然使用pip进行安装，pip install rospkg

5. 使用gazebo进行仿真的时候，因为之前安装了anaconda python3.5 导致在编译的过程中出现_tf2 import error. 要切回ubuntu自带的python2.7才可以
系统自带的python位于/usr/bin, 所以先查看echo $PATH, 如果/usr/bin位于anaconda之前，则python会先找到python2.7而不是anaconda。
我的方法是将固定的PATH在.bashrc中先死打印，之后调整anaconda的前后顺序，这样就可以避免PATH出现一堆重复的路径。


# gazebo world 加载失败黑屏的问题
$cd /usr/share/gazebo-7/worlds

$gazebo willowgarage.world 

打开gazebo的时候会发现一直处于这种状态，这是因为model库加载不正确导致的。

解决办法：

$ cd ~/

$ hg clone https://bitbucket.org/osrf/gazebo_models

下载完成后将gazebo_models复制到~/.gazebo文件夹中，重命名为models.
注意使用cp 进行文件复制的时候，将该目录下的所有文件复制是
cp -r dir/ dir/
查看models下是否包含所有的model文件





