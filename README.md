# demo_ws

moveit2_humble源码安装： https://moveit.picknik.ai/humble/index.html

运行 moveit_task_constructor demo：
在工作空间下运行：
ros2 launch rviz_only_moveit_config_manual move_group.launch.py
另一个窗口运行：
ros2 launch hello_moveit_task_constructor run.launch.py ‘exe:=<filename>’
替换 filename 为 hello_moveit_task_constructor 功能包 src 文件下文件名

