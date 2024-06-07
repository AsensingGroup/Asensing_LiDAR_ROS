# 更改日志

## Asensing Driver SDK V1.0.0

- 升级到 V1.0.0 正式发布版本
- 支持 OpenMP 编译特性
- 设置队列内存限制，超出后直接丢弃数据，并抛出异常
- 更新 A2 标定文件默认参数

## Asensing Driver SDK V0.3.4

- 修复使用在线雷达时间戳时的时区问题
- 优化线程名称设置，支持多个线程实例共用

## Asensing Driver SDK V0.3.3

- 设置默认不使用雷达时间戳
- 优化三角函数计算
- 优化 A0 点云解析中的回波判断逻辑
- 支持从 CMake 配置是否提高线程优先级
- 增加基于 Frame ID 的分帧策略，并应用于 A0 解析器
- 将基于 PCL 消息中的 intensity 字段从 float 修改为 std::uint8_t

## Asensing Driver SDK V0.3.2

- 在线模式支持增大 socket 接收缓冲区大小
- 增加线程优先级设置，并提高 recv_packet 和 process_packet 线程优先级
- 针对 A2 激光雷达调整解析过程，以兼容现阶段的 A2 激光雷达

## Asensing Driver SDK V0.3.1

- 增加 PointXYZIRT 的扩展字段
- 默认不等待 DIFOP 报文

## Asensing Driver SDK V0.3.0

- 支持距离、角度、灰度、模组等多维度点云过滤
- 支持行列型点云消息格式
- 支持紧凑型点云消息格式
- 支持 A2 激光雷达角度补偿
- 更新 A2 激光雷达标定文件
- 修复 Windows 兼容问题

## Asensing Driver SDK V0.2.3

- A2 点云解析器增加水平角度偏移参数配置

## Asensing Driver SDK V0.2.2

- 支持 A2 点云协议
- 更新 json 库（移除 cJSON）

## Asensing Driver SDK V0.2.1

- 优化 pcap 解析卡顿问题
