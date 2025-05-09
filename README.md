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

遇到如下问题
[component_container_isolated-1] [ERROR] [1745403788.489409154] [controller_server]: Node '/controller_server' has already been added to an executor.
原因是在nav2 插件里

# 3.编写python控制器节点

我遇到了conda环境与ros2系统之间的交互问题，表现为找不到conda环境安装的包。
解决方法是将环境变量PYTHONPATH中加入conda环境中的python包路径
如下：
        export PYTHONPATH="$PYTHON_SITE_PACKAGES:$PYTHONPATH"
        export PYTHONPATH=/home/feiyu/NeuPAN:$PYTHONPATH

遇到了matplotlib 冲突问题，NeuPAN环境中和系统自带的版本冲突,删除环境中的matplotlib
遇到了numpy版本问题，numpy版本为2.2不适配，在环境中删除之后安装numpy 1.23