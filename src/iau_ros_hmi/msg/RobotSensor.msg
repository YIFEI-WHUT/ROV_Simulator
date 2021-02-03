time         stamp        # 消息时间戳

float32      temperature  # 温度
float32[8]   ultrasonic   # 超声波测距距离
float32      left_speed   # 左轮速度
float32      right_speed  # 右轮速度
float32      pitch        # 姿态 pitch
float32      roll         # 姿态 roll
float32      yaw          # 姿态 yaw
float32[3]   accel        # 加速度 3轴
float32[3]   gyro         # 角速度 3轴
float32[3]   magnet       # 磁通量 3轴
int32        ul_control   # 是否启用下位机超声波急停控制
int32        ul_status    # 下位机是否处于超声波急停状态
bool         embtn_on     # 急停按钮 on/off
bool         emrmt_on     # 遥控急停 on/off
bool         rmtctl       # 遥控控制 on/off
float32      left_turn    # 左轮转速增量，全圆比率，上次数据时刻至今
float32      right_turn   # 右轮转速增量，全圆比率，上次数据时刻至今
float32      left_delta   # 左轮距离增量 m
float32      right_delta  # 右轮距离增量 m
float32      voltage      # 电池电压

