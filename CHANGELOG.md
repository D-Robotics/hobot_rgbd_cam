# Changelog for package rgbd_sensor

tros_2.1.0 (2024-04-01)
------------------
1. 适配ros2 humble零拷贝。
2. 新增中英双语README。
3. 零拷贝通信使用的qos的Reliability由RMW_QOS_POLICY_RELIABILITY_RELIABLE（rclcpp::QoS()）变更为RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT（rclcpp::SensorDataQoS()）。

tros_2.0.4 (2024-3-6)
------------------
1. 配置文件路径由`/opt/tros/lib`变更为`/opt/tros/${TROS_DISTRO}/lib`。

tros_2.0.3 (2024-1-18)
------------------
1. 更新image step的不对齐问题。

tros_2.0.2 (2023-12-22)
------------------
1. 更新package.xml，解决arm平台打包失败问题。

tros_2.0.1 (2023-06-09)
------------------
1. 修改depth数据发布来源，解决深度数据数值错误问题

tros_2.0.0 (2023-05-11)
------------------
1. 更新package.xml，支持应用独立打包
2. 适配新版本X3派

hhp_1.0.6RC1 (2022-09-09)
------------------
1. 新增CP3AM型号摄像头的标定文件
2. 将默认的相机标定文件读取路径更换为绝对路径
3. 增加launch文件中的参数：相机标定文件读取路径

hhp_1.0.6 (2022-08-30)
------------------
1. RGBD sensor完成x3派适配，可用转接板接到x3派上使用
2. 新增读取相机标定文件并发布相机内参的功能

v1.0.2 (2022-07-20)
------------------
1. 合入最新RGBD厂家SDK
2. 使用opencv接口优化NV12转RGB数据耗时，提升整体输出帧率

v1.0.1 (2022-06-23)
------------------
1. 解决 rgbd 通过shared_mem 传输存在的问题