std_msgs/Bool EPS_En_Status  # 0: Disabled, 1: Enabled
std_msgs/Int8 EPS_Control_Board_Status  # Value [0, 2]
std_msgs/Int8 EPS_Control_Status  # Value [0, 15]
std_msgs/Bool EPS_USER_CAN_ERR  # 0: No Error, 1: Error
std_msgs/Bool EPS_ERR  # 0: No Error, 1: Error
std_msgs/Bool EPS_Veh_CAN_ERR  # 0: No Error, 1: Error
std_msgs/Bool EPS_SAS_ERR  # 0: No Error, 1: Error
std_msgs/Bool Override_Ignore_Status  # 0: Not Ignored, 1: Ignored
std_msgs/Bool Override_Status  # 0: No Override, 1: Override
std_msgs/Float32 StrAng  # Steering angle, scaled 0.1 (deg)
std_msgs/Float32 Str_Drv_Tq  # Steering driver torque, scaled 0.01
std_msgs/Float32 Str_Out_Tq  # Steering output torque, scaled 0.1
std_msgs/Int8 EPS_Alive_Cnt  # Alive counter, value [0, 255]
std_msgs/Bool ACC_En_Status  # True: Enabled, False: Disabled
std_msgs/Int8 ACC_Control_Board_Status  # Value [0, 7]
std_msgs/Int8 ACC_Control_Status  # Value [0, 15]
std_msgs/Bool ACC_USER_CAN_ERR  # False: No Error, True: Error
std_msgs/Bool ACC_ERR  # False: No Error, True: Error
std_msgs/Int8 VS  # Vehicle speed (km/h)
std_msgs/Float32 Long_Accel  # Longitudinal acceleration, scaled 0.01 (m/s^2)
std_msgs/Bool Hazard_En  # True: Enabled, False: Disabled
std_msgs/Bool Turn_Left_En  # True: Enabled, False: Disabled
std_msgs/Bool Turn_Right_En  # True: Enabled, False: Disabled
std_msgs/Bool ACC_Veh_ERR  # False: No Error, True: Error
std_msgs/Int8 G_SEL_DISP  # Gear selector display
std_msgs/Int8 ACC_Alive_Cnt  # Alive counter, value [0, 255]
std_msgs/Float32 WHEEL_SPD_FR  # Front right wheel speed, scaled 0.03125 (kph)
std_msgs/Float32 WHEEL_SPD_FL  # Front left wheel speed, scaled 0.03125 (kph)
std_msgs/Float32 WHEEL_SPD_RR  # Rear right wheel speed, scaled 0.03125 (kph)
std_msgs/Float32 WHEEL_SPD_RL  # Rear left wheel speed, scaled 0.03125 (kph)
std_msgs/Float32 LAT_ACCEL  # Lateral acceleration, scaled 0.01 (m/s^2)
std_msgs/Float32 YAW_RATE  # Yaw rate, scaled 0.01 (deg/s)
std_msgs/Float32 BRK_CYLINDER  # Brake cylinder pressure, scaled 0.1
std_msgs/Float32 Long_ACCEL  # Longitudinal acceleration, scaled 0.01 (m/s^2)