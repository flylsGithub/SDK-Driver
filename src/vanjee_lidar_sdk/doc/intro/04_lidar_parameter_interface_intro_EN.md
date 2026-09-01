# 4 Introduction to Lidar Parameters Interface

**Support lidar type:**
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

## 4.1 Lidar parameter interface topic information

```
std_msgs/Header header
uint16 cmd_id
uint8 cmd_type
uint32 repeat_interval
string data
```

``` header ```
lidar parameter interface topic header information:
ROS1 includes: sequence number(seq), timestamp (timestamp), coordinate system (frame id);
ROS2 includes: timestamp(timestamp),coordinate system (frame id)

``` cmd_id ```
Command identification:
0x0001 - work mode;         (Support lidar type: vanjee_722, vanjee_722d, vanjee_722f, vanjee_722h, vanjee_722z)
0x0002 - temperature;       (Support lidar type: vanjee_722d, vanjee_722f, vanjee_722h, vanjee_722z)
0x0003 - firmware version;  (Support lidar type: vanjee_716mini, vanjee_718h, vanjee_719, vanjee_719c, vanjee_719e, vanjee_720_16, vanjee_720_32, vanjee_722, vanjee_722d, vanjee_722f, vanjee_722h, vanjee_722z)
0x0004 - sn;                (Support lidar type: vanjee_720_16, vanjee_720_32, vanjee_722, vanjee_722d, vanjee_722f, vanjee_722h, vanjee_722z)
0x0005 - acceleration range;(Support lidar type: vanjee_722d, vanjee_722z)
0x0006 - zone alarm;        (Support lidar type: vanjee_718h, vanjee_719e)
0x0007 - alarm zone switch; (Support lidar type: vanjee_718h)
0x0008 - basic parameters;        (Support lidar type: vanjee_716mini, vanjee_718h, vanjee_719e, vanjee_720_16, vanjee_722)
0x000b - time sync mode;          (Support lidar type: vanjee_722)
0x000c - flash data read/write;   (Support lidar type: vanjee_722d)
0x000d - LD enable;               (Support lidar type: vanjee_722d)
0x0100 - lidar state;       (Support lidar type: vanjee_716mini, vanjee_718h, vanjee_719, vanjee_719e, vanjee_720_16, vanjee_720_32, vanjee_722, vanjee_722d, vanjee_722f, vanjee_722h, vanjee_722z, vanjee_750, vanjee_760)

``` cmd_type ```
Command type:
the parameters of "set lidar work mode" include:
0 - query (Support cmd id: 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x000b)
1 - set   (Support cmd id: 0x0001)
Note: The lidar state driver will proactively report and does not require querying.

``` repeat_interval ```
Instruction loop sending interval, unit: milliseconds (ms). The parameter must be configured as an integer multiple of 100 ms:
0 -  single query (one-time only);
1000 - loop query (every 1000ms);

``` data ```
Detailed parameters:
{
  "temperature": 20.0
}
Note: The data is encapsulated in JSON format.

``` lidar parameter interface topic header information
  uint32 seq = 0;
  float64 timestamp = 0.0;
  string frame_id = "";
```

## 4.2 Lidar parameter interface information

data structure:
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
application program interface:
void lidarParameterApi(const LidarParameterInterface& lidar_param);
``` 
device class:
LidarDriver<PointCloudMsg> driver;

## 4.2.1 Receive the responded data of lidar parameter command
Register callback function:
driver.regLidarParameterInterfaceCallback(allocateLidarParameterInterfaceMemoryCallback, lidarParameterInterfaceCallback);

Sdk store the responded data of lidar parameter command into the memory provided by the allocateLidarParameterInterfaceMemoryCallback function
Receive the responded data of lidar parameter command by lidarParameterInterfaceCallback function

## 4.2.2 Send lidar parameter command
example of lidar parameter command:
LidarParameterInterface lidar_param;
lidar_param.cmd_id = 2;
lidar_param.cmd_type = 0;
lidar_param.repeat_interval = 0;
lidar_param.data = "";

Call interface:
driver.lidarParameterApi(lidar_param);

**refer the demo for the specific implementation method**

## 4.3 Lidar parameter commands for all lidars

### 4.3.1 WLR-716mini

cmd_id: 0x0003 -Get lidar firmware version (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -The feedback of getting lidar firmware version (to user)
cmd_type: 0
repeat_interval: 0
data:"{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0008 -Get basic parameters of lidar(to lidar)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0008 -The feedback of getting basic parameters(to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "lidar_type": "WLR-716Mini",
  "mac": "FF-AA-00-00-00-00",
  "firmware_version": "v1.0"
}"

cmd_id: 0x0100 -Lidar state reporting (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001,
  "fault_state": 0      (0-Normal 1-Warning 2-Fault)
}"

### 4.3.2 WLR-718h

cmd_id: 0x0003 -Get lidar firmware version (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -The feedback of getting lidar firmware version (to user)
cmd_type: 0
repeat_interval: 0
data:"{
  "firmware_version": "v1.0"
}"


cmd_id: 0x0006 - Alarm zone information reporting (to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "alarm_zone":
  {
    "io": [1, 2, 3]
  }
}"

cmd_id: 0x0007 - Get alarm zone information (to lidar)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0007 - The feedback of getting alarm zone information (to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "bank_id": 1
}"

cmd_id: 0x0007 - Set alarm zone (to lidar)：
cmd_type: 1
repeat_interval: 0
data: "{
  "bank_id": 1
}"

cmd_id: 0x0007 - The feedback of getting alarm zone information (to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "bank_id": 1
}"

cmd_id: 0x0008 -Get basic parameters of lidar(to lidar)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0008 -The feedback of getting basic parameters(to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "lidar_type": "WLR-718H",
  "mac": "FF-AA-00-00-00-00",
  "firmware_version": "v1.0"
}"

cmd_id: 0x0100 -Lidar state reporting (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001,
  "fault_state": 0       (0-Normal 1-Warning 2-Fault)
}"

### 4.3.3 WLR-719

cmd_id: 0x0003 -Get lidar firmware version (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -The feedback of getting lidar firmware version (to user)
cmd_type: 0
repeat_interval: 0
data:"{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0100 -Lidar state reporting (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001,
  "fault_state": 0      (0-Normal 1-Warning 2-Fault)
}"

### 4.3.4 WLR-719C

cmd_id: 0x0003 -Get lidar firmware version (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -The feedback of getting lidar firmware version (to user)
cmd_type: 0
repeat_interval: 0
data:"{
  "firmware_version": "v1.0"
}"

### 4.3.5 WLR-719E

cmd_id: 0x0003 -Get lidar firmware version (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -The feedback of getting lidar firmware version (to user)
cmd_type: 0
repeat_interval: 0
data:"{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0006 -Alarm zone information reporting (to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "alarm_zone":
  {
    "ossd": [1, 2],                             (Value on intrusion, OSSD index)
    "io": [1, 2, 3, 4],                         (Value on intrusion, IO index)
    "ossd_a_1": 0,                              (0-Low 1-High -1-Unused)
    "ossd_b_1": 0,                              (0-Low 1-High -1-Unused)
    "input_a": [1, 0, -1, -1, -1, -1, -1, -1],  (0-Low 1-High -1-Unused, index corresponds to IN input number)
    "input_b": [1, 0, -1, -1, -1, -1, -1, -1],  (0-Low 1-High -1-Unused, index corresponds to IN input number)
    "in_io_a": [-1, -1, -1, -1],                (0-Low 1-High -1-Unused, index corresponds to IO input number)
    "in_io_b": [-1, -1, -1, -1],                (0-Low 1-High -1-Unused, index corresponds to IO input number)
    "out_io": [0, 0, 0, 0],                     (0-Low 1-High -1-Unused, index corresponds to IO output number)
    "intrusion_info_arm1": [1],                 (Value on intrusion, corresponding to ARM1 shutdown path index)
    "input_speed_arm1": [0, 0],                 (ARM1 dynamic input speed mm/s)
    "intrusion_info_arm2": [],                  (Value on intrusion, corresponding to ARM2 shutdown path index)
    "input_speed_arm2": [0, 0]                  (ARM2 dynamic input speed mm/s)
  }
}"

cmd_id: 0x0008 -Get basic parameters of lidar(to lidar)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0008 -The feedback of getting basic parameters(to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "lidar_type": "WLR-719E",
  "mac": "FF-AA-00-00-00-00",
  "firmware_version": "v1.0"
}"

cmd_id: 0x0009 -Get field data of lidar(to lidar)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0009 -The feedback of getting field data(to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "fields":[
    {
      "field_data_id": 1,                (The field data id)
      "line_id" : 1                      (The field line id, uint32_t)
      "range_max": [0, 0, 2, 3, 0...],   (Collection of max distance data in this field, float32)
      "range_min": [0, 0, 0, 0, 0...],   (Collection of min distance data in this field, float32)
      "start_angle": 2.408554367,        (Current field start angle, rad, float32)
      "angle_increment" : -0.003488888,  (Angle increment, rad，float32)
      "angular_resolution": 0.003488888, (Angular resolution, rad，float32)
      "field_definition": 1              (2-protected field 1-Warning field 0-Non, uint32_t)
    },
    {
      "field_data_id": 2,                 (The field data id)
      "line_id" : 1                       (The field line id, uint32_t)
      "range_max": [0, 0, 0, 3.1, 3.2...],(Collection of distance data in this field, float32)
      "range_min": [0, 0, 0, 0, 0...],    (Collection of min distance data in this field, float32)
      "start_angle": 2.408554367,         (Current field start angle, rad, float32)
      "angle_increment" : -0.003488888,   (Angle increment, rad，float32)
      "angular_resolution": 0.003488888,  (Angular resolution, rad，float32)
      "field_definition": 2               (2-protected field 1-Warning field 0-Non, uint32_t)
    }
  ]
}"

cmd_id: 0x000a -Get monitoring case of lidar(to lidar)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x000a -The feedback of getting monitoring case(to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "monitoring_case": [
    {
      "monitoring_case_list_id: " 1,  (The monitoring case list id, uint32_t)
      "monitoring_case_id": 1,        (The monitoring case id, uint32_t)
      "monitoring_case_type": 0       (Monitoring case type: 0-N_to_1, 1-Complementary Sampling, 2-Speed Range, 3-Speed Limit, 4-N_to_1+Speed Range, 5-N_to_1+Speed Limit, 6-Complementary Sampling+Speed Range, 7-Complementary Sampling+Speed Limit, -1-Unused, int32_t)
      "input_io": [-1, -1, -1, -1],               (IO configuration triggered by this monitoring case, 0-Low 1-High -1-Unused)
      "input_in": [1, 0, -1, -1, -1, -1, -1, -1], (IN Configuration Triggered by This Monitoring Case, 0-Low 1-High -1-Unused)
      "range": [],                   	            (null)
      ""output_io": [[-1, -1, -1, -1, -1, -1], .., [-1, -1, -1, -1, -1, -1]]	(Intrusion paths 1~8 in sequence; each group outputs OSSD1, OSSD2, IO1, IO2, IO3, IO4 sequentially)
      "case_info": [1, ..., 8]        (Intrusion paths 1~8 in sequence; Field id range 1~128, 0-permanently off, 255-permanently on, uint32_t)
    },
    {
      "monitoring_case_list_id: " 1,  (The monitoring case list id, uint32_t)
      "monitoring_case_id": 1,        (The monitoring case id, uint32_t)
      "monitoring_case_type": 1       (Monitoring case type: 0-N_to_1, 1-Complementary Sampling, 2-Speed Range, 3-Speed Limit, 4-N_to_1+Speed Range, 5-N_to_1+Speed Limit, 6-Complementary Sampling+Speed Range, 7-Complementary Sampling+Speed Limit, -1-Unused, int32_t)
      "input_io": [-1, -1],           (IO configuration triggered by this monitoring case, 0-Low 1-High 2-Low/High -1-Unused)
      "input_in": [1, 0, -1, -1],     (IN configuration triggered by this monitoring case, 0-Low 1-High 2-Low/High -1-Unused)
      "range": [],                    (null)
      ""output_io": [[-1, -1, -1, -1, -1, -1], .., [-1, -1, -1, -1, -1, -1]]	(Intrusion paths 1~8 in sequence; each group outputs OSSD1, OSSD2, IO1, IO2, IO3, IO4 sequentially)
      "case_info": [1, ..., 0]        (Intrusion paths 1~8 in sequence; Field id range 1~128, 0-permanently off, 255-permanently on, uint32_t)
    },
    {
      "monitoring_case_list_id: " 1,  (The monitoring case list id, uint32_t)
      "monitoring_case_id": 1,        (The monitoring case id, uint32_t)
      "monitoring_case_type": 2       (Monitoring case type: 0-N_to_1, 1-Complementary Sampling, 2-Speed Range, 3-Speed Limit, 4-N_to_1+Speed Range, 5-N_to_1+Speed Limit, 6-Complementary Sampling+Speed Range, 7-Complementary Sampling+Speed Limit, -1-Unused, int32_t)
      "input_io": [],                 (null)
      "input_in": [],                 (null)
      "range": [-100, 100],           (Monitoring case trigger range, [min value, max value])
      ""output_io": [[-1, -1, -1, -1, -1, -1], .., [-1, -1, -1, -1, -1, -1]]	(Intrusion paths 1~8 in sequence; each group outputs OSSD1, OSSD2, IO1, IO2, IO3, IO4 sequentially)
      "case_info": [1, ..., 255]      (Intrusion paths 1~8 in sequence; Field id range 1~128, 0-permanently off, 255-permanently on, uint32_t)
    },
    {
      "monitoring_case_list_id: " 1,  (The monitoring case list id, uint32_t)
      "monitoring_case_id": 1,        (The monitoring case id, uint32_t)
      "monitoring_case_type": 3       (Monitoring case type: 0-N_to_1, 1-Complementary Sampling, 2-Speed Range, 3-Speed Limit, 4-N_to_1+Speed Range, 5-N_to_1+Speed Limit, 6-Complementary Sampling+Speed Range, 7-Complementary Sampling+Speed Limit, -1-Unused, int32_t)
      "input_io": [],                 (null)
      "input_in": [],                 (null)
      "range": [-100, 100],           (Monitoring case trigger range, [min value, max value])
      ""output_io": [[-1, -1, -1, -1, -1, -1], .., [-1, -1, -1, -1, -1, -1]]	(Intrusion paths 1~8 in sequence; each group outputs OSSD1, OSSD2, IO1, IO2, IO3, IO4 sequentially)
      "case_info": [1, ..., 8]        (Intrusion paths 1~8 in sequence; Field id range 1~128, 0-permanently off, 255-permanently on, uint32_t)
    },
    {
      "monitoring_case_list_id: " 1,  (The monitoring case list id, uint32_t)
      "monitoring_case_id": 1,        (The monitoring case id, uint32_t)
      "monitoring_case_type": 4       (Monitoring case type: 0-N_to_1, 1-Complementary Sampling, 2-Speed Range, 3-Speed Limit, 4-N_to_1+Speed Range, 5-N_to_1+Speed Limit, 6-Complementary Sampling+Speed Range, 7-Complementary Sampling+Speed Limit, -1-Unused, int32_t)
      "input_io": [0, 1, -1, -1],                 (IO configuration triggered by this monitoring case, 0-Low 1-High -1-Unused)
      "input_in": [1, 0, -1, -1, -1, -1, -1, -1], (IN configuration triggered by this monitoring case, 0-Low 1-High -1-Unused)
      "range": [-100, 100],           (Monitoring case trigger range, [min value, max value])
      ""output_io": [[-1, -1, -1, -1, -1, -1], .., [-1, -1, -1, -1, -1, -1]]	(Intrusion paths 1~8 in sequence; each group outputs OSSD1, OSSD2, IO1, IO2, IO3, IO4 sequentially)
      "case_info": [1, ..., 255]      (Intrusion paths 1~8 in sequence; Field id range 1~128, 0-permanently off, 255-permanently on, uint32_t)
    },
    {
      "monitoring_case_list_id: " 1,  (The monitoring case list id, uint32_t)
      "monitoring_case_id": 1,        (The monitoring case id, uint32_t)
      "monitoring_case_type": 6       (Monitoring case type: 0-N_to_1, 1-Complementary Sampling, 2-Speed Range, 3-Speed Limit, 4-N_to_1+Speed Range, 5-N_to_1+Speed Limit, 6-Complementary Sampling+Speed Range, 7-Complementary Sampling+Speed Limit, -1-Unused, int32_t)
      "input_io": [-1, -1],           (IO configuration triggered by this monitoring case, 0-Low 1-High 2-Low/High -1-Unused)
      "input_in": [1, 0, -1, -1],     (IN configuration triggered by this monitoring case, 0-Low 1-High 2-Low/High -1-Unused)
      "range": [-100, 100],           (Monitoring case trigger range, [min value, max value])
      ""output_io": [[-1, -1, -1, -1, -1, -1], .., [-1, -1, -1, -1, -1, -1]]	(Intrusion paths 1~8 in sequence; each group outputs OSSD1, OSSD2, IO1, IO2, IO3, IO4 sequentially)
      "case_info": [1, ..., 0]        (Intrusion paths 1~8 in sequence; Field id range 1~128, 0-permanently off, 255-permanently on, uint32_t)
    }
  ]
}"

cmd_id: 0x0100-0x010f -Lidar state reporting (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001,
  "fault_state": 0      (0-Normal 1-Warning 2-Fault)
}"

### 4.3.6 WLR-720_16

cmd_id: 0x0003 -Get lidar firmware version (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -The feedback of getting lidar firmware version (to user)
cmd_type: 0
repeat_interval: 0
data:"{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -Get lidar sn (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -The feedback of getting lidar sn (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0008 -Get basic parameters of lidar(to lidar)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0008 -The feedback of getting basic parameters(to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "lidar_type": "WLR-720",
  "sn": "123456",
  "firmware_version": "v1.0"
}"

cmd_id: 0x0100 -Lidar state reporting (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001,
  "fault_state": 0      (0-Normal 1-Warning 2-Fault)
}"

### 4.3.7 WLR-720_32

cmd_id: 0x0003 -Get lidar firmware version (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -The feedback of getting lidar firmware version (to user)
cmd_type: 0
repeat_interval: 0
data:"{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -Get lidar sn (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -The feedback of getting lidar sn (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0100 -Lidar state reporting (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001,
  "fault_state": 0      (0-Normal 1-Warning 2-Fault)
}"

### 4.3.8 WLR-722

cmd_id: 0x0001 -Get lidar work mode (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0001 -The feedback of getting lidar work mode command (to user)
cmd_type: 1
repeat_interval: 0
data: "{
  "work_mode": 0  (0-working; 1-standby; 2-only open channels 25-28)
}"

cmd_id: 0x0001 -Set lidar work mode (to lidar)
cmd_type: 1
repeat_interval: 0
data: "{
  "work_mode": 0  (0-working; 1-standby; 2-only open channels 25-28)
}"

cmd_id: 0x0001 -The feedback of getting lidar work mode command (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0  (0-working; 1-standby; 2-only open channels 25-28)
}"

cmd_id: 0x0003 -Get lidar firmware version (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -The feedback of getting lidar firmware version (to user)
cmd_type: 0
repeat_interval: 0
data:"{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -Get lidar sn (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -The feedback of getting lidar sn (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0008 -Get basic parameters of lidar(to lidar)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0008 -The feedback of getting basic parameters(to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "lidar_type": "WLR-722",
  "difop_version": 1,
  "work_mode": 0,               (0-working; 1-standby; 2-mode1)
  "sn": "123456",
  "firmware_version": "v1.0",
  "emission_module_temp": 20.0, (degree Celsius)
  "imu_state": 0,               (0-Normal 1-Abnormal)
  "imu_type": 0,                (0-IMU42688 1-IMU42586)
  "motor_state": 1,             (0-Stop 1-Rotate)
  "motor_speed": 600,           (r/min)
  "clock_source": 1,            (0-Free clock 1-GPS 2-PTP 3-NTP)
  "gps_state": 3                (0-Disconnected 1-Message Abnormal 2-PPS Abnormal 3-Normal)
}"

cmd_id: 0x000b -Get time sync mode（to lidar)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x000b -The feedback of getting time sync mode (to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "clock_source": 0,            (0-Free clock 1-GPS 2-PTP 3-NTP)
  "utc_flag": 1,                (1-UTC)
  "utc_ts": 1735660800,
  "time_mode": 0,               (0-UTC 1-UNIX 2-UNIX_V2)
  "gps_name_sentence": 0,       (0-GNRMC 1-GNGGA)
  "ptp_ieee": 0,                (0-IEEE1588V2)
  "ptp_transport": 0,           (0-UDP/IP)
  "ptp_domain_num": 0,
  "ptp_announce_interval": 1,   (Transmission interval of PTP Announce messages / s)
  "ptp_sync_interval": 1,       (Transmission interval of PTP Sync messages / s)
  "ntp_ip" = [192, 168, 2, 88],
  "ntp_sync_interval": 10       (Transmission interval of NTP Sync messages / s)
}"

cmd_id: 0x000b -Set time sync mode（to lidar)：
cmd_type: 1
repeat_interval: 0
data: "{
  "clock_source": 0,          (0-Free clock)
  "parse_flag": 0,            (0-Do not parse 1-Parse, When set to 0, the following content will not be parsed. Configuration can be omitted and default values will be applied)
  "utc_flag": 1,              (1-UTC)
  "utc_ts": 1735660800,
  "time_mode": 0              (0-UTC 1-UNIX 2-UNIX_V2)
}"
OR
cmd_id: 0x000b -Set time sync mode（to lidar)：
cmd_type: 1
repeat_interval: 0
data: "{
  "clock_source": 1,          (1-GPS)
  "parse_flag": 0,            (0-Do not parse 1-Parse, When set to 0, the following content will not be parsed. Configuration can be omitted and default values will be applied)
  "gps_name_sentence": 0      (0-GNRMC 1-GNGGA)
}"
OR
cmd_id: 0x000b -Set time sync mode（to lidar)：
cmd_type: 1
repeat_interval: 0
data: "{
  "clock_source": 2,          (2-PTP)
  "parse_flag": 0,            (0-Do not parse 1-Parse, When set to 0, the following content will not be parsed. Configuration can be omitted and default values will be applied)
  "ptp_ieee": 0,              (0-IEEE1588V2)
  "ptp_transport": 0,         (0-UDP/IP)
  "ptp_domain_num": 0,
  "ptp_announce_interval": 1, (Transmission interval of PTP Announce messages / s)
  "ptp_sync_interval": 1      (Transmission interval of PTP Sync messages / s)
}"
OR
cmd_id: 0x000b -Set time sync mode（to lidar)：
cmd_type: 1
repeat_interval: 0
data: "{
  "clock_source": 3,            (3-NTP)
  "parse_flag": 0,              (0-Do not parse 1-Parse, When set to 0, the following content will not be parsed. Configuration can be omitted and default values will be applied)
  "ntp_ip" = [192, 168, 2, 88],
  "ntp_sync_interval": 10       (Transmission interval of NTP Sync messages / s)
}"

cmd_id: 0x000b -The feedback of setting time sync mode (to user)：
cmd_type: 1
repeat_interval: 0
data: "{
  "state":  0 (0-failed 1-succ)
}"

cmd_id: 0x0100 -Lidar state reporting (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001,
  "fault_state": 0      (0-Normal 1-Warning 2-Fault)
}"

### 4.3.9 WLR-722d

cmd_id: 0x0001 -Get lidar work mode (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0001 -The feedback of getting lidar work mode command (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0  (0-working 1-standby)
}"

cmd_id: 0x0001 -Set lidar work mode (to lidar)
cmd_type: 1
repeat_interval: 0
data: "{
  "work_mode": 0  (0-working 1-standby)
}"

cmd_id: 0x0001 -The feedback of getting lidar work mode command (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "state": 0 (0-failed 1-succ)
}"

cmd_id: 0x0002 -Get lidar temperature (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0002 -The feedback of getting lidar temperature (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "temperature": 20.5
}"

cmd_id: 0x0003 -Get lidar firmware version (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -The feedback of getting lidar firmware version (to user)
cmd_type: 0
repeat_interval: 0
data:"{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -Get lidar sn (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -The feedback of getting lidar sn (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0005 -Get range of acceleration (to lidar):
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0005 -The feedback of getting range of acceleration (to user):
cmd_type: 0
repeat_interval: 0
data: "{
  "acceleration_range": 0 (0-±2G 1-±4G)
}"

cmd_id: 0x0005 -Set range of acceleration (to lidar):
cmd_type: 1
repeat_interval: 0
data: "{
  "acceleration_range": 0 (0-±2G 1-±4G)
}"

cmd_id: 0x0005 -The feedback of getting range of acceleration (to user):
cmd_type: 0
repeat_interval: 0
data: "{
  "acceleration_range": 0 (0-±2G 1-±4G)
}"

cmd_id: 0x0008 -Lidar basic parameters reporting(to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "firmware_version": "v1.0",
  "sn": "123456",
  "emission_module_state": 0,           (0-off 1-full brightness)
  "emission_module_temperature": 20.5,
  "motor_state": 1,                     (0-stop 1-start)
  "motor_speed": 450,                   (r/min)
  "imu_acc_range": 0,                   (0-±2G 1-±4G)
  "gps_state": 0                        (0-normal 1-fault)
}"

cmd_id: 0x000c -Get Flash data（to lidar)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x000c -The feedback of getting Flash data(to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "state": 0,               (0-error data 1-succ)
  "data": [0, 0, 0, ..., 0] (Flash data is uint8_t, size of 100 bytes)
}"

cmd_id: 0x000c -Set Flash data（to lidar)：
cmd_type: 1
repeat_interval: 0
data: "{
  "data": [0, 0, 0, ..., 0] (Flash data is uint8_t with a size ≤ 100 bytes, assigned starting from index 0 by default)
}"

cmd_id: 0x000c -The feedback of setting Flash data(to user)：
cmd_type: 1
repeat_interval: 0
data: "{
  "state": 0  (0-failed 1-succ)
}"

cmd_id: 0x000d -light enable（to lidar)：
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x000d -The feedback of enable LD (to user)：
cmd_type: 0
repeat_interval: 0
data: "{
  "state": 1  (0-failed 1-succ)
}"

cmd_id: 0x0100 -Lidar state reporting (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001,
  "fault_state": 0      (0-Normal 1-Warning 2-Shut down)
}"

### 4.3.10 WLR-722f

cmd_id: 0x0001 -Get lidar work mode (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0001 -The feedback of getting lidar work mode command (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0  (0-working 1-standby)
}"

cmd_id: 0x0001 -Set lidar work mode (to lidar)
cmd_type: 1
repeat_interval: 0
data: "{
  "work_mode": 0  (0-working 1-standby)
}"

cmd_id: 0x0001 -The feedback of getting lidar work mode command (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0  (0-working 1-standby)
}"

cmd_id: 0x0002 -Get lidar temperature (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0002 -The feedback of getting lidar temperature (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "temperature": 20.5
}"

cmd_id: 0x0003 -Get lidar firmware version (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -The feedback of getting lidar firmware version (to user)
cmd_type: 0
repeat_interval: 0
data:"{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -Get lidar sn (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -The feedback of getting lidar sn (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0100 -Lidar state reporting (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001, (fault code ,each bit corresponds to a fault. If you need to confirm the fault type, please refer to the lidar product manual)
  "fault_state": 1      (1-Warning 2-Fault)
}"

### 4.3.11 WLR-722h

cmd_id: 0x0001 -Get lidar work mode (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0001 -The feedback of getting lidar work mode command (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0  (0-working 1-standby)
}"

cmd_id: 0x0001 -Set lidar work mode (to lidar)
cmd_type: 1
repeat_interval: 0
data: "{
  "work_mode": 0  (0-working 1-standby)
}"

cmd_id: 0x0001 -The feedback of getting lidar work mode command (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0  (0-working 1-standby)
}"

cmd_id: 0x0002 -Get lidar temperature (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0002 -The feedback of getting lidar temperature (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "temperature": 20.5
}"

cmd_id: 0x0003 -Get lidar firmware version (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -The feedback of getting lidar firmware version (to user)
cmd_type: 0
repeat_interval: 0
data:"{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -Get lidar sn (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -The feedback of getting lidar sn (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0100 -Lidar state reporting (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001, (fault code, each bit corresponds to a fault. If you need to confirm the fault type, please refer to the lidar product manual)
  "fault_state": 1      (1-Warning 2-Fault)
}"

### 4.3.12 WLR-722z

cmd_id: 0x0001 -Get lidar work mode (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0001 -The feedback of getting lidar work mode command (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "work_mode": 0  (0-working 1-standby)
}"

cmd_id: 0x0001 -Set lidar work mode (to lidar)
cmd_type: 1
repeat_interval: 0
data: "{
  "work_mode": 0  (0-working 1-standby)
}"

cmd_id: 0x0001 -The feedback of setting lidar work mode command (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "state": 0  (0-failed 1-succ)
}"

cmd_id: 0x0002 -Get lidar temperature (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0002 -The feedback of getting lidar temperature (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "temperature": 20.5
}"

cmd_id: 0x0003 -Get lidar firmware version (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0003 -The feedback of getting lidar firmware version (to user)
cmd_type: 0
repeat_interval: 0
data:"{
  "firmware_version": "v1.0"
}"

cmd_id: 0x0004 -Get lidar sn (to lidar)
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0004 -The feedback of getting lidar sn (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "sn": "123456"
}"

cmd_id: 0x0005 -Get range of acceleration (to lidar):
cmd_type: 0
repeat_interval: 0
data: ""

cmd_id: 0x0005 -The feedback of getting range of acceleration (to user):
cmd_type: 0
repeat_interval: 0
data: "{
  "acceleration_range": 0  (0-±2G 1-±4G)
}"

cmd_id: 0x0005 -Set range of acceleration (to lidar):
cmd_type: 1
repeat_interval: 0
data: "{
  "acceleration_range": 0  (0-±2G 1-±4G)
}"

cmd_id: 0x0005 -The feedback of getting range of acceleration (to user):
cmd_type: 0
repeat_interval: 0
data: "{
  "acceleration_range": 0  (0-±2G 1-±4G)
}"

cmd_id: 0x0100 -Lidar state reporting (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001,
  "fault_state": 0      (0-Normal 1-Warning 2-Shut down)
}"

### 4.3.13 WLR-750

cmd_id: 0x0100-0x010f -Lidar state reporting (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "fault_code": 0x0001,
  "fault_state": 0      (0-Normal 1-Warning 2-Fault)
}"

### 4.3.14 WLR-760

cmd_id: 0x0100 -Lidar state reporting (to user)
cmd_type: 0
repeat_interval: 0
data: "{
  "over_voltage_alarm": 1,    (0-Abnormal 1-Normal)
  "under_voltage_alarm": 1,   (0-Abnormal 1-Normal)
  "high_temperature_alarm": 1 (0-Abnormal 1-Normal)
}"
