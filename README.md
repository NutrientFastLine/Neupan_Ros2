# 基于NeuPAN的ROS2包

# 先决条件
1.ROS2 humble
2.Python >=3.10
3.安装NeuPAN,有其对应的conda环境

# 1.编写通信接口
ros2 pkg create neupan_interfaces --build-type ament_cmake --dependencies geometry_msgs sensor_msgs std_msgs

CMakeLists.txt文件中
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NeupanCommand.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

package.xml文件中
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

# 2.编写导航控制器插件
ros2 pkg create nav2_neupan_controller --build-type ament_cmake --dependencies pluginlib nav2_core
编写neupan_controller.hpp和neupan_controller.cpp文件
编写nav2_neupan_controller.xml文件
编写CmakeLists.txt文件
编写package.xml

# 3.编写python控制器节点

我遇到了conda环境与ros2系统之间的交互问题，表现为找不到conda环境安装的包。
解决方法是将环境变量PYTHONPATH中加入conda环境中的python包路径
如下：
        export PYTHONPATH="$PYTHON_SITE_PACKAGES:$PYTHONPATH"
        export PYTHONPATH=/home/feiyu/NeuPAN:$PYTHONPATH

