# 工训赛代码

##### 主控:

​	stm32h7

##### FreeRTOS任务一览:

###### System_Task: 负责控制闪灯以及其他基础操作(例如使用野火调试工具)

###### IMU_Task: 负责使用yaw积分解算BMI088陀螺仪读取到的数据

###### Arm_Task: 根据其他任务下达的指令作为一个机械臂控制的中间层, 解算后下发指令到RS485_Task任务进行控制

###### Car_Task: 对小车直接进行实时的控制, 内部包含了CAN的操作

###### RS485_Task: 对485总线进行管理, 进行超时控制, CRC验证及发送失败的重新发送等功能, 使总线经过包装后变得可靠

###### RASPI_Task: 负责进行与树莓派进行USB通讯, 其中树莓派类似于一个服务器, 32向树莓派发送读取的请求后树莓派经过处理然后回复32并返回视觉计算出的数据

###### Path_Task: 指挥各个模块顺序执行工训赛流程中的一系列任务, 比如先移动, 扫描抓物体.....







PS:
Load文件夹内的所有内容以及RASPI的部分内容都是我队友写的, 主要是一些线性流程, 代码风格可能有些割裂, 请见谅!



















