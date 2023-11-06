[toc]
# 工程说明
* 工程组织 lidar_3D_proj文件夹
    * summit_xl_common 功能包集
    * 
    * robot_upper_app 功能包集 机器人周边/上层应用
        * 语音识别/交流
        * 视觉识别/物体识别/对象检测跟踪
        * 机器学习

# 硬件配置说明
* 参考：[Summit-XL系列移动平台](https://zhuanlan.zhihu.com/p/91415812)
* Summit-XL系列移动机器人包括Summit-XL、Summit-XL HL、Summit-XL Steel三款移动机器人平台；SUMMIT-XL有基于4个高功率马达轮子的转向运动学。每个车轮集成了一个轮毂无刷电机与变速箱和编码器(可选)。
* SUMMIT-XL有两种可能的运动学配置： 全向配置安装麦克纳姆轮、传统的车轮(轮辋安装)。
* 搭载PTZ摄像机，可进行行远程操作。{云台全方位（左右/上下）移动及镜头变倍、变焦控制}
* 机械参数：（有三种结构，不同）
    * 可以承载重物(最高可达65公斤)
    * 尺寸： 722*613*392 mm 
    * 重量45kg~105kg
    * 负载 20～130kg
    * 速度 3m/s
    * 使用时间： 5小时～10小时
    * 牵引电机4*250～500W
    * 最大爬坡：38～16度
    * 控制器： intel j1900 （选配inntel i7）
    * 传感器选配： Hokuyo 激光雷达、IMU、PTZ相机、GPS、ORBBEC 3D相机


# 文件组织结构
* lidar_3D_proj
    * summit_xl_sim 仿真环境
        * sxl_gazebo 包
            * 用来启动不同世界/机器人环境。

        * summit_xl_sim 启动仿真环境gazebo
        * sxl_sim 自定义的各种仿真launch文件,比如slam/vslam/3d-laserSlam仿真,navigation仿真,gui命令等.
            * > 这里只是本项目的机器人启动控制**launch文件**,以及相关的**配置文件**. 其依赖的包/具体实现可能是在各个地方.
            * 比如gui命令, 其具体使用的是tv_gui包中的gui(因为在实际使用中这个包在tv_gui项目中进行改进/维护)
    * summit_xl_common:  具体的slam/navigation实现,仿真或者实际使用时依靠这些包
        * sxl_vslam 包
        * sxl_loam 


# sxl_xx 自定义机器人使用基本步骤
1.  启动 gazebo仿真环境
    ```
    roslaunch sxl_gazebo sxl_one_robot_world.launch
    ```
2. 启动rviz可视化
    ```
    roslaunch sxl_sim sxl_rviz.launch
    ```
    * 单独的一个包sxl_sim，而不方到sxl_gazebo包中，便于和仿真完全分离(仿真和实际调试都可以使用)
3. 启动应用
    ```
    roslaunch sxl_navigation sxl_slam.launch # 建图
    ```
* 保存地图
    ```
    rosrun map_server map_saver map:=map -f  path
    eg:
    rosrun map_server map_saver map:=/robot/map -f /home/jl/catkin_ws/src/lidar_3D_proj/summit_xl_sim/map/simple_workshop

    ```
    * 注意是节点map_saver， map为topic 

# summit_xl 机器人(源码)使用
* > 基本原则,尽量不更改源码,自己需要的话重新写launch文件.
* 参考
    * [RobotsSummitXL-wiki](Thttp://wiki.ros.org/Robots/SummitXL)
* 源码文件组织结构:
    * summit_xl_common:
        * summit_xl_description: urdf/meshes描述
        * summit_xl_control: (在sim的wiki中介绍的))生成gazebo中的控制器(用以结合gazebo仿真)
        * summit_xl_robot_control 没有???(查看wiki) 
        * 
    * summit_xl_sim
        *  summit_xl_sim_bringup
    * robotnik_sensors: r
    * robotnik_msgs:
* 源文件/开源工程具体使用:
    * 参照源码的readme.md
    * 启动:
        ```
        roslaunch summit_xl_sim_bringup summit_xl_complete.launch
        ```
        * 可以根据参数设置, 实现gmapping /navigation功能; 但自己使用时,只用它完成启动world,其他如gmapping/navigation, 以及rviz等用自己的启动文件.


# lidar_3D_proj 工程的使用
* > 自己更改的包的使用：
* 基本思路：
    * 仿真环境包
        * 启动模拟环境代替物理模型；
    * 其他包基本就是真实环境和仿真环境共用的包
        * 即，在物理环境时，不启动这个仿真包即可完全一样的运行。
    * 所以：
        1. gazebo环境启动，用以模拟/代替真实模型和环境（包括了所有传感器信息）。
        2. 启动rviz，可视化信息； rivz单独启动，仿真和真实调试环境都需要(部署环境不需要)
        3. 启动实际业务程序(要做到：仿真和实际一样)。

* 基本文件/结构组织：
    * sxl_gazebo: 
        * 机器人模型，以及机器人底盘相关的数据： 里程计、imu、激光雷达等。
        * > sxl_gazeb包，只负责启动gazebo相关环境即可(模型)； rviz以及仿真放到sxl_sim包
    1. 启动模型
        ```
        roslaunch sxl_gazebo sxl_robot_world.launch
        ```
        * 需要正确启动模型，模型中包括传感器以及控制器，注意看有没有报错。比如，模型中使用了diff_drive_controller/DiffDriveController，那么需要有这个控制插件，才能实现差速控制。
            * 当然，这个插件很常用，就在ros_controllers包中。
        * 测试：
            ```
            rostopic pub -r 10 /robot/robotnik_base_control/cmd_vel ...
            ```
            * 比如发布原地转圈的cmd_vel，即可看到模型运动。
    2. 启动rviz可视化
        ```
        roslaunch sxl_sim sxl_rviz.launch
        ```
    3. 启动自己的任务
        * 比如slam，路径规划等
    * gazebo模型启动完成后，可以看到它本身的tf关系树是完整的，自然是没有odom坐标系的，因为它只是发布了自身传感器数据。 现在，需要自己处理传感器数据，比如编码器和imu进行融合得到里程计从而发布出里程计数据(使用tf发布到tf树)。
        * 可以使用 robot_pose_ekf 包进行融合，得到里程计数据(并发布tf)

    

     
