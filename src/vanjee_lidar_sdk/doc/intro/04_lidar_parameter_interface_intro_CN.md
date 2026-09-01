# 4 雷达参数接口介绍

**支持雷达型号:**
  WLR-716mini
  WLR-718h
  WLR-719
  WLR-719e
  WLR-720_16
  WLR-720_32
  WLR-722
  WLR-722d
  WLR-722f
  WLR-722h
  WLR-722z
  WLR-750
  WLR-760

## 4.1 雷达参数接口话题信息

```
std_msgs/Header header
uint16 cmd_id
uint8 cmd_type
uint32 repeat_interval
string data
```

``` header ```
雷达参数接口话题头信息:
ROS1包含内容有:序号(seq) 时间戳(timestamp) 坐标系(frame id);
ROS2包含内容有:时间戳(timestamp) 坐标系(frame id).

``` cmd_id ```
操作码:
0x0001 - 雷达工作模式;    (支持雷达型号: vanjee_722, vanjee_722d, vanjee_722f, vanjee_722h, vanjee_722z)
0x0002 - 雷达温度;        (支持雷达型号: vanjee_722d, vanjee_722f, vanjee_722h, vanjee_722z)
0x0003 - 雷达固件版本号;  (支持雷达型号: vanjee_716mini, vanjee_718h, vanjee_719, vanjee_719c, vanjee_719e, vanjee_720_16, vanjee_720_32, vanjee_722, vanjee_722d, vanjee_722f, vanjee_722h, vanjee_722z)
0x0004 - 雷达序列号;      (支持雷达型号: vanjee_720_16, vanjee_720_32, vanjee_722, vanjee_722d, vanjee_722f, vanjee_722h, vanjee_722z)
0x0005 - 加速度量程;      (支持雷达型号: vanjee_722d, vanjee_722z)
0x0006 - 区域报警;       (支持雷达型号: vanjee_718h, vanjee_719e)
0x0007 - 报警区域切换;    (支持雷达型号: vanjee_718h)
0x0008 - 雷达基本参数;    (支持雷达型号: vanjee_716mini, vanjee_718h, vanjee_719e, vanjee_720_16, vanjee_722)
0x000b - 时间同步模式;    (支持雷达型号: vanjee_722)
0x000c - Flash数据读/写; (支持雷达型号: vanjee_722d)
0x000d - 发光使能;       (支持雷达型号: vanjee_722d)
0x0100 - 雷达状态;       (支持雷达型号: vanjee_716mini, vanjee_718h, vanjee_719, vanjee_719e, vanjee_720_16, vanjee_720_32, vanjee_722, vanjee_722d, vanjee_722f, vanjee_722h, vanjee_722z, vanjee_750, vanjee_760)

``` cmd_type ```
"操作码类型:
0 - 查询  (支持操作码: 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x000b)
1 - 设置  (支持操作码: 0x0001)
备注: 雷达状态驱动会主动上报，不需要查询.

``` repeat_interval ```
指令循环发送间隔, 单位：毫秒(ms)，参数需配置100ms的整数倍:
0 - 单次查询;
1000 - 循环查询, 1000ms 查询一次;

``` data ```
详细参数:
{
  "temperature": 20.0
}
备注: 数据封装为 JSON 格式.

``` 雷达参数接口话题头信息
  uint32 seq = 0;
  float64 timestamp = 0.0;
  string frame_id = "";
```

## 4.2 雷达参数接口信息

数据结构:
class LidarParameterInterface {
 public:
  uint32 seq;
  float64 timestamp;
  uint16 cmd_id;
  uint8 cmd_type;
  uint32 repeat_interval;
  std::string data;
};

``` 
接口函数:
void lidarParameterApi(const LidarParameterInterface& lidar_param);
``` 
设备类:
LidarDriver<PointCloudMsg> driver;

## 4.2.1 接收雷达参数指令的响应数据
回调函数注册:
driver.regLidarParameterInterfaceCallback(allocateLidarParameterInterfaceMemoryCallback, lidarParameterInterfaceCallback);
sdk通过 allocateLidarParameterInterfaceMemoryCallback 提供的内存空间存储雷达参数指令的响应数据
通过 lidarParameterInterfaceCallback 接收雷达参数指令的响应数据

## 4.2.2 发送雷达参数设置指令
雷达参数指令示例:
LidarParameterInterface lidar_param;
lidar_param.cmd_id = 2;
lidar_param.cmd_type = 0;
lidar_param.repeat_interval = 0;
lidar_param.data = "";


接口调用:
driver.lidarParameterApi(lidar_param);

**具体实现方法可参考demo用例**

## 4.3 各型号雷达参数指令说明

### 4.3.1 WLR-716mini

cmd_id: 0x0003 -获取雷达固件版本号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -回复雷达固件版本号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0008 -获取雷达基本参数(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0008 -回复雷达基本参数(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "lidar_type": "WLR-716Mini",
  "mac": "FF-AA-00-00-00-00",
  "firmware_version": "v1.0"
}"

cmd_id: 0x0100 -雷达运行状态上报(上行)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001, (故障码)
  "fault_state": 0      (0-正常 1-警告 2-故障)
}"

### 4.3.2 WLR-718h

cmd_id: 0x0003 -获取雷达固件版本号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -回复雷达固件版本号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0006 - 区域报警信息上报(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "alarm_zone":
  {
    "io": [1, 2, 3]
  }
}"

cmd_id: 0x0007 - 获取报警区域信息(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0007 - 报警区域信息上报(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "bank_id": 1
}"

cmd_id: 0x0007 - 设置报警区域(下行)：
cmd_type: 1
repeat_interval: 0
data: "{
  "bank_id": 1
}"

cmd_id: 0x0007 - 报警区域信息上报(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "bank_id": 1
}"

cmd_id: 0x0008 -获取雷达基本参数(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0008 -回复雷达基本参数(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "lidar_type": "WLR-718H",
  "mac": "FF-AA-00-00-00-00",
  "firmware_version": "v1.0"
}"

cmd_id: 0x0100 -雷达运行状态上报(上行)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001, (故障码)
  "fault_state": 0      (0-正常 1-警告 2-故障)
}"

### 4.3.3 WLR-719

cmd_id: 0x0003 -获取雷达固件版本号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -回复雷达固件版本号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0100 -雷达运行状态上报(上行)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001, (故障码)
  "fault_state": 0      (0-正常 1-警告 2-故障)
}"

### 4.3.4 WLR-719C

cmd_id: 0x0003 -获取雷达固件版本号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -回复雷达固件版本号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0"
}"

### 4.3.5 WLR-719E

cmd_id: 0x0003 -获取雷达固件版本号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -回复雷达固件版本号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0006 - 区域报警信息上报(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "alarm_zone":
  {
    "ossd": [1, 2],                             (闯入时有值, OSSD序号)
    "io": [1, 2, 3, 4],                         (闯入时有值, IO序号)
    "ossd_a_1": 0,                              (0-低电平 1-高电平 -1-未使用)
    "ossd_b_1": 0,                              (0-低电平 1-高电平 -1-未使用)
    "input_a": [1, 0, -1, -1, -1, -1, -1, -1],  (0-低电平 1-高电平 -1-未使用 索引对应IN输入序号)
    "input_b": [1, 0, -1, -1, -1, -1, -1, -1],  (0-低电平 1-高电平 -1-未使用 索引对应IN输入序号)
    "in_io_a": [-1, -1, -1, -1],                (0-低电平 1-高电平 -1-未使用 索引对应IO输入序号)
    "in_io_b": [-1, -1, -1, -1],                (0-低电平 1-高电平 -1-未使用 索引对应IO输入序号)
    "out_io": [0, 0, 0, 0],                     (0-低电平 1-高电平 -1-未使用 索引对应IO输出序号)
    "intrusion_info_arm1": [1],                 (闯入时有值, 对应ARM1关断路径序号)
    "input_speed_arm1": [0, 0],                 (ARM1动态输入速度 mm/s)
    "intrusion_info_arm2": [],                  (闯入时有值, 对应ARM2关断路径序号)
    "input_speed_arm2": [0, 0]                  (ARM2动态输入速度 mm/s)
  }
}"

cmd_id: 0x0008 -获取雷达基本参数(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0008 -回复雷达基本参数(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "lidar_type": "WLR-719E",
  "mac": "FF-AA-00-00-00-00",
  "firmware_version": "v1.0"
}"

cmd_id: 0x0009 -获取雷达区域数据(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0009 -回复雷达区域数据(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "fields":[
    {
      "field_data_id": 1,                (区域 id)
      "line_id" : 1                      (区域线号, uint32_t)
      "range_max": [0, 0, 2, 3, 0...],   (区域最大距离值集合, float32)
      "range_min": [0, 0, 0, 0, 0...],   (区域最小距离值集合, float32)
      "start_angle": 2.408554367,        (区域起始角度, rad, float32)
      "angle_increment" : -0.003488888,  (角度递增值, rad，float32)
      "angular_resolution": 0.003488888, (角度分辨率, rad，float32)
      "field_definition": 1              (区域定义: 2-保护区域 1-警告区域 0-无, uint32_t)
    },
    {
      "field_data_id": 2,                 (区域 id)
      "line_id" : 1                       (区域线号, uint32_t)
      "range_max": [0, 0, 0, 3.1, 3.2...],(区域最大距离值集合, float32)
      "range_min": [0, 0, 0, 0, 0...],    (区域最小距离值集合, float32)
      "start_angle": 2.408554367,         (区域起始角度, rad, float32)
      "angle_increment" : -0.003488888,   (角度递增值, rad，float32)
      "angular_resolution": 0.003488888,  (角度分辨率, rad，float32)
      "field_definition": 2               (区域定义: 2-保护区域 1-警告区域 0-无, uint32_t)
    }
  ]
}"

cmd_id: 0x000a -获取雷达监控事例(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x000a -回复雷达监控事例(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "monitoring_case": [
    {
      "monitoring_case_list_id: " 1,  (监控事例列表编号, uint32_t)
      "monitoring_case_id": 1,        (监控事例编号, uint32_t)
      "monitoring_case_type": 0       (监控事例类型: 0-N_to_1 1-互补采样 2-速度范围 3-速度极限 4-N_to_1+速度范围 5-N_to_1+速度极限 6-互补采样+速度范 围 7-互补采样+速度极限 -1-未使用, int32_t)
      "input_io": [-1, -1, -1, -1],               (监控事例触发 IO 配置, 0-低电平 1-高电平 -1-未使用)
      "input_in": [1, 0, -1, -1, -1, -1, -1, -1], (监控事例触发 IN 配置, 0-低电平 1-高电平 -1-未使用)
      "range": [],                   	            (空)
      ""output_io": [[-1, -1, -1, -1, -1, -1], .., [-1, -1, -1, -1, -1, -1]]	(依次未关断路径1~8, 每组输出依次为 OSSD1 OSSD2 IO1 IO2 IO3 IO4)
      "case_info": [1, ..., 8]        (依次未关断路径1~8, 1~128-区域id 0-始终关 255-始终开, uint32_t)
    },
    {
      "monitoring_case_list_id: " 1,  (监控事例列表编号, uint32_t)
      "monitoring_case_id": 1,        (监控事例编号, uint32_t)
      "monitoring_case_type": 1       (监控事例类型: 0-N_to_1 1-互补采样 2-速度范围 3-速度极限 4-N_to_1+速度范围 5-N_to_1+速度极限 6-互补采样+速度范 围 7-互补采样+速度极限 -1-未使用, int32_t)
      "input_io": [-1, -1],           (监控事例触发 IO 配置, 0-低电平 1-高电平 2-低电平/高电平 -1-未使用)
      "input_in": [1, 0, -1, -1],     (监控事例触发 IN 配置, 0-低电平 1-高电平 2-低电平/高电平 -1-未使用)
      "range": [],                    (空)
      ""output_io": [[-1, -1, -1, -1, -1, -1], .., [-1, -1, -1, -1, -1, -1]]	(依次未关断路径1~8, 每组输出依次为 OSSD1 OSSD2 IO1 IO2 IO3 IO4)
      "case_info": [1, ..., 0]        (依次未关断路径1~8, 1~128-区域id 0-始终关 255-始终开, uint32_t)
    },
    {
      "monitoring_case_list_id: " 1,  (监控事例列表编号, uint32_t)
      "monitoring_case_id": 1,        (监控事例编号, uint32_t)
      "monitoring_case_type": 2       (监控事例类型: 0-N_to_1 1-互补采样 2-速度范围 3-速度极限 4-N_to_1+速度范围 5-N_to_1+速度极限 6-互补采样+速度范 围 7-互补采样+速度极限 -1-未使用, int32_t)
      "input_io": [],                 (空)
      "input_in": [],                 (空)
      "range": [-100, 100],           (监控事例触发范围, [最小值, 最大值])
      ""output_io": [[-1, -1, -1, -1, -1, -1], .., [-1, -1, -1, -1, -1, -1]]	(依次未关断路径1~8, 每组输出依次为 OSSD1 OSSD2 IO1 IO2 IO3 IO4)
      "case_info": [1, ..., 255]      (依次未关断路径1~8, 1~128-区域id 0-始终关 255-始终开, uint32_t)
    },
    {
      "monitoring_case_list_id: " 1,  (监控事例列表编号, uint32_t)
      "monitoring_case_id": 1,        (监控事例编号, uint32_t)
      "monitoring_case_type": 3       (监控事例类型: 0-N_to_1 1-互补采样 2-速度范围 3-速度极限 4-N_to_1+速度范围 5-N_to_1+速度极限 6-互补采样+速度范 围 7-互补采样+速度极限 -1-未使用, int32_t)
      "input_io": [],                 (空)
      "input_in": [],                 (空)
      "range": [-100, 100],           (监控事例触发范围, [最小值, 最大值])
      ""output_io": [[-1, -1, -1, -1, -1, -1], .., [-1, -1, -1, -1, -1, -1]]	(依次未关断路径1~8, 每组输出依次为 OSSD1 OSSD2 IO1 IO2 IO3 IO4)
      "case_info": [1, ..., 8]        (依次未关断路径1~8, 1~128-区域id 0-始终关 255-始终开, uint32_t)
    },
    {
      "monitoring_case_list_id: " 1,  (监控事例列表编号, uint32_t)
      "monitoring_case_id": 1,        (监控事例编号, uint32_t)
      "monitoring_case_type": 4       (监控事例类型: 0-N_to_1 1-互补采样 2-速度范围 3-速度极限 4-N_to_1+速度范围 5-N_to_1+速度极限 6-互补采样+速度范 围 7-互补采样+速度极限 -1-未使用, int32_t)
      "input_io": [0, 1, -1, -1],                 (监控事例触发 IO 配置, 0-低电平 1-高电平 -1-未使用)
      "input_in": [1, 0, -1, -1, -1, -1, -1, -1], (监控事例触发 IN 配置, 0-低电平 1-高电平 -1-未使用)
      "range": [-100, 100],           (监控事例触发范围, [最小值, 最大值])
      ""output_io": [[-1, -1, -1, -1, -1, -1], .., [-1, -1, -1, -1, -1, -1]]	(依次未关断路径1~8, 每组输出依次为 OSSD1 OSSD2 IO1 IO2 IO3 IO4)
      "case_info": [1, ..., 255]      (依次未关断路径1~8, 1~128-区域id 0-始终关 255-始终开, uint32_t)
    },
    {
      "monitoring_case_list_id: " 1,  (监控事例列表编号, uint32_t)
      "monitoring_case_id": 1,        (监控事例编号, uint32_t)
      "monitoring_case_type": 6       (监控事例类型: 0-N_to_1 1-互补采样 2-速度范围 3-速度极限 4-N_to_1+速度范围 5-N_to_1+速度极限 6-互补采样+速度范 围 7-互补采样+速度极限 -1-未使用, int32_t)
      "input_io": [-1, -1],           (监控事例触发 IO 配置, 0-低电平 1-高电平 2-低电平/高电平 -1-未使用)
      "input_in": [1, 0, -1, -1],     (监控事例触发 IN 配置, 0-低电平 1-高电平 2-低电平/高电平 -1-未使用)
      "range": [-100, 100],           (监控事例触发范围, [最小值, 最大值])
      ""output_io": [[-1, -1, -1, -1, -1, -1], .., [-1, -1, -1, -1, -1, -1]]	(依次未关断路径1~8, 每组输出依次为 OSSD1 OSSD2 IO1 IO2 IO3 IO4)
      "case_info": [1, ..., 0]        (依次未关断路径1~8, 1~128-区域id 0-始终关 255-始终开, uint32_t)
    }
  ]
}"

cmd_id: 0x0100-0x010f -雷达运行状态上报(上行)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001, (故障码)
  "fault_state": 0      (0-正常 1-警告 2-故障)
}"

### 4.3.6 WLR-720_16

cmd_id: 0x0003 -获取雷达固件版本号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -回复雷达固件版本号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -获取雷达SN号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -回复雷达SN号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0008 -获取雷达基本参数(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0008 -回复雷达基本参数(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "lidar_type": "WLR-720",
  "sn": "123456",
  "firmware_version": "v1.0"
}"

cmd_id: 0x0100 -雷达运行状态上报(上行)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001, (故障码)
  "fault_state": 0      (0-正常 1-警告 2-故障)
}"

### 4.3.7 WLR-720_32

cmd_id: 0x0003 -获取雷达固件版本号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -回复雷达固件版本号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -获取雷达SN号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -回复雷达SN号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0100 -雷达运行状态上报(上行)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001, (故障码)
  "fault_state": 0      (0-正常 1-警告 2-故障)
}"

### 4.3.8 WLR-722
cmd_id: 0x0001 -获取雷达运行模式（下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0001 -回复雷达运行模式(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0  (0-工作模式 1-待机模式 2-只开启25-28通道)
}"

cmd_id: 0x0001 -设置雷达运行模式（下行)：
cmd_type: 1
repeat_interval: 0
data: "{
  "work_mode": 0  (0-工作模式 1-待机模式 2-只开启25-28通道)
}"

cmd_id: 0x0001 -回复雷达运行模式(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0  (0-工作模式 1-待机模式 2-只开启25-28通道)
}"

cmd_id: 0x0003 -获取雷达固件版本号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -回复雷达固件版本号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -获取雷达SN号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -回复雷达SN号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0008 -获取雷达基本参数(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0008 -回复雷达基本参数(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "lidar_type": "WLR-722",
  "difop_version": 1,
  "work_mode": 0,               (0-工作模式 1-待机模式 2-只开启25-28通道)
  "sn": "123456",
  "firmware_version": "v1.0",
  "emission_module_temp": 20.0, (摄氏度)
  "imu_state": 0,               (0-正常 1-异常)
  "imu_type": 0,                (0-IMU42688 1-IMU42586)
  "motor_state": 1,             (0-停止 1-启动)
  "motor_speed": 600,           (r/min)
  "clock_source": 1,            (0-自由时钟 1-GPS 2-PTP 3-NTP)
  "gps_state": 3                (0-断开连接 1-报文异常 2-PPS异常 3-正常)
}"

cmd_id: 0x000b -获取时间同步模式（下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x000b -回复时间同步模式(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "clock_source": 0,            (0-自由时钟 1-GPS 2-PTP 3-NTP)
  "utc_flag": 1,                (1-UTC)
  "utc_ts": 1735660800,
  "time_mode": 0,               (0-UTC 1-UNIX 2-UNIX_V2)
  "gps_name_sentence": 0,       (0-GNRMC 1-GNGGA)
  "ptp_ieee": 0,                (0-IEEE1588V2)
  "ptp_transport": 0,           (0-UDP/IP)
  "ptp_domain_num": 0,
  "ptp_announce_interval": 1,   (PTP公告报文发送时间间隔/s)
  "ptp_sync_interval": 1,       (PTP时间同步报文发送时间间隔/s)
  "ntp_ip" = [192, 168, 2, 88],
  "ntp_sync_interval": 10       (NTP时间同步报文发送时间间隔/s)
}"

cmd_id: 0x000b -设置时间同步模式（下行)：
cmd_type: 1
repeat_interval: 0
data: "{
  "clock_source": 0,          (0-自由时钟)
  "parse_flag": 0,            (0- 不解析 1-解析, 设置为 0 时以下内容不解析, 可不做配置, 使用默认值)
  "utc_flag": 1,              (1-UTC)
  "utc_ts": 1735660800,
  "time_mode": 0              (0-UTC 1-UNIX 2-UNIX_V2)
}"
或
cmd_id: 0x000b -设置时间同步模式（下行)：
cmd_type: 1
repeat_interval: 0
data: "{
  "clock_source": 1,          (1-GPS)
  "parse_flag": 0,            (0- 不解析 1-解析, 设置为 0 时以下内容不解析, 可不做配置, 使用默认值)
  "gps_name_sentence": 0      (0-GNRMC 1-GNGGA)
}"
或
cmd_id: 0x000b -设置时间同步模式（下行)：
cmd_type: 1
repeat_interval: 0
data: "{
  "clock_source": 2,          (2-PTP)
  "parse_flag": 0,            (0- 不解析 1-解析, 设置为 0 时以下内容不解析, 可不做配置, 使用默认值)
  "ptp_ieee": 0,              (0-IEEE1588V2)
  "ptp_transport": 0,         (0-UDP/IP)
  "ptp_domain_num": 0,
  "ptp_announce_interval": 1, (PTP公告报文发送时间间隔/s)
  "ptp_sync_interval": 1      (PTP时间同步报文发送时间间隔/s)
}"
或
cmd_id: 0x000b -设置时间同步模式（下行)：
cmd_type: 1
repeat_interval: 0
data: "{
  "clock_source": 3,            (3-NTP)
  "parse_flag": 0,              (0- 不解析 1-解析, 设置为 0 时以下内容不解析, 可不做配置, 使用默认值)
  "ntp_ip" = [192, 168, 2, 88],
  "ntp_sync_interval": 10       (NTP时间同步报文发送时间间隔/s)
}"

cmd_id: 0x000b -回复时间同步模式(上行)：
cmd_type: 1
repeat_interval: 0
data: "{
  "state": 0 (0-失败 1-成功)
}"

cmd_id: 0x0100 -雷达运行状态上报(上行)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001,   (故障码)
  "fault_state": 0        (0-正常 1-警告 2-故障)
}"

### 4.3.9 WLR-722d

cmd_id: 0x0001 -获取雷达运行模式（下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0001 -回复雷达运行模式(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0 (0-工作模式 1-待机模式)
}"

cmd_id: 0x0001 -设置雷达运行模式（下行)：
cmd_type: 1
repeat_interval: 0
data: "{
  "work_mode": 0 (0-工作模式 1-待机模式)
}"

cmd_id: 0x0001 -回复雷达运行模式(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "state": 0 (0-设置失败 1-设置成功)
}"

cmd_id: 0x0002 -获取雷达温度（下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0002 -回复雷达温度(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "temperature": 20.5
}"

cmd_id: 0x0003 -获取雷达固件版本号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -回复雷达固件版本号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -获取雷达SN号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -回复雷达SN号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0005 -获取加速度量程(下行):
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0005 -回复加速度量程(上行):
cmd_type: 0
repeat_interval: 0
data: "{
  "acceleration_range": 0 (0-±2G 1-±4G)
}"

cmd_id: 0x0005 -设置加速度量程(下行):
cmd_type: 1
repeat_interval: 0
data: "{
  "acceleration_range": 0 (0-±2G 1-±4G)
}"

cmd_id: 0x0005 -回复加速度量程(上行):
cmd_type: 0
repeat_interval: 0
data: "{
  "acceleration_range": 0 (0-±2G 1-±4G)
}"

cmd_id: 0x0008 -雷达基本参数上报(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0",
  "sn": "123456",
  "emission_module_state": 0,           (0-不发光 1-全发光)
  "emission_module_temperature": 20.5,
  "motor_state": 1,                     (0-停止 1-启动)
  "motor_speed": 450,                   (r/min)
  "imu_acc_range": 0,                   (0-±2G 1-±4G)
  "gps_state": 0                        (0-正常 1-异常)
}"

cmd_id: 0x000c -获取 Flash 数据（下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x000c -回复 Flash 数据(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "state": 0,               (0-获取数据异常 1-获取成功)
  "data": [0, 0, 0, ..., 0] (Flash 数据 100 字节 uint8_t)
}"

cmd_id: 0x000c -设置 Flash 数据（下行)：
cmd_type: 1
repeat_interval: 0
data: "{
  "data": [0, 0, 0, ..., 0] (Flash 数据 <=100 字节 uint8_t, 默认从索引 0 开始赋值)
}"

cmd_id: 0x000c -回复 Flash 数据(上行)：
cmd_type: 1
repeat_interval: 0
data: "{
  "state": 0  (0-设置失败 1-设置成功)
}"

cmd_id: 0x000d -发光使能（下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0100 -雷达运行状态上报(上行)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001-故障码
  "fault_state": 0 (0-正常 1-警告 2-关机)
}"

### 4.3.10 WLR-722f

cmd_id: 0x0001 -获取雷达运行模式（下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0001 -回复雷达运行模式(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0 (0-工作模式 1-待机模式)
}"

cmd_id: 0x0001 -设置雷达运行模式（下行)：
cmd_type: 1
repeat_interval: 0
data: "{
  "work_mode": 0 (0-工作模式 1-待机模式)
}"

cmd_id: 0x0001 -回复雷达运行模式(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0 (0-工作模式 1-待机模式)
}"

cmd_id: 0x0002 -获取雷达温度（下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0002 -回复雷达温度(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "temperature": 20.5
}"

cmd_id: 0x0003 -获取雷达固件版本号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -回复雷达固件版本号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -获取雷达SN号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -回复雷达SN号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0100 -雷达运行状态上报(上行)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001, (故障码, 每个bit对应一个故障，如果需要确认故障类型可参考雷达产品手册)
  "fault_state": 1      (1-警告 2-故障)
}"

### 4.3.11 WLR-722h

cmd_id: 0x0001 -获取雷达运行模式（下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0001 -回复雷达运行模式(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0 (0-工作模式 1-待机模式)
}"

cmd_id: 0x0001 -设置雷达运行模式（下行)：
cmd_type: 1
repeat_interval: 0
data: "{
  "work_mode": 0 (0-工作模式 1-待机模式)
}"

cmd_id: 0x0001 -回复雷达运行模式(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0 (0-工作模式 1-待机模式)
}"

cmd_id: 0x0002 -获取雷达温度（下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0002 -回复雷达温度(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "temperature": 20.5
}"

cmd_id: 0x0003 -获取雷达固件版本号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -回复雷达固件版本号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -获取雷达SN号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -回复雷达SN号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0100 -雷达运行状态上报(上行)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001, (故障码, 每个bit对应一个故障，如果需要确认故障类型可参考雷达产品手册)
  "fault_state": 1      (1-警告 2-故障)
}"

### 4.3.12 WLR-722z

cmd_id: 0x0001 -获取雷达运行模式（下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0001 -回复雷达运行模式(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0 (0-工作模式 1-待机模式)
}"

cmd_id: 0x0001 -设置雷达运行模式（下行)：
cmd_type: 1
repeat_interval: 0
data: "{
  "work_mode": 0 (0-工作模式 1-待机模式)
}"

cmd_id: 0x0001 -回复雷达运行模式(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "state": 1    (0-设置失败 1-设置成功)
}"

cmd_id: 0x0002 -获取雷达温度（下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0002 -回复雷达温度(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "temperature": 20.5
}"

cmd_id: 0x0003 -获取雷达固件版本号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -回复雷达固件版本号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -获取雷达SN号(下行)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -回复雷达SN号(上行)：
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0005 -获取加速度量程(下行):
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0005 -回复加速度量程(上行):
cmd_type: 0
repeat_interval: 0
data: "{
  "acceleration_range": 0 (0-±2G 1-±4G)
}"

cmd_id: 0x0005 -设置加速度量程(下行):
cmd_type: 1
repeat_interval: 0
data: "{
  "acceleration_range": 0 (0-±2G 1-±4G)
}"

cmd_id: 0x0005 -回复加速度量程(上行):
cmd_type: 0
repeat_interval: 0
data: "{
  "acceleration_range": 0 (0-±2G 1-±4G)
}"

cmd_id: 0x0100 -雷达运行状态上报(上行)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001, (故障码)
  "fault_state": 0      (0-正常 1-警告 2-关机)
}"

### 4.3.13 WLR-750

cmd_id: 0x0100-0x010f -雷达运行状态上报(上行)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001, (故障码)
  "fault_state": 0      (0-正常 1-警告 2-故障)
}"

### 4.3.14 WLR-760

cmd_id: 0x0100 -雷达运行状态上报(上行)
cmd_type: 0
repeat_interval: 0
data: "{
  "over_voltage_alarm": 1,      (0-异常 1-正常)
  "under_voltage_alarm": 1,     (0-异常 1-正常)
  "high_temperature_alarm": 1   (0-异常 1-正常)
}"
