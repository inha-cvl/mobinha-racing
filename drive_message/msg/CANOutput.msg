std_msgs/String EPS_En_Status  # 0: Disabled, 1: Enabled
std_msgs/String EPS_Control_Board_Status  # Value [0, 2]
std_msgs/String EPS_Control_Status  # Value [0, 15]
std_msgs/String EPS_USER_CAN_ERR  # 0: No Error, 1: Error
std_msgs/String EPS_ERR  # 0: No Error, 1: Error
std_msgs/String EPS_Veh_CAN_ERR  # 0: No Error, 1: Error
std_msgs/String EPS_SAS_ERR  # 0: No Error, 1: Error
std_msgs/String Override_Ignore_Status  # 0: Not Ignored, 1: Ignored
std_msgs/String Override_Status  # 0: No Override, 1: Override
std_msgs/String StrAng  # Steering angle, scaled 0.1 (deg)
std_msgs/String Str_Drv_Tq  # Steering driver torque, scaled 0.01
std_msgs/String Str_Out_Tq  # Steering output torque, scaled 0.1
std_msgs/String EPS_Alive_Cnt  # Alive counter, value [0, 255]
std_msgs/String ACC_En_Status  # True: Enabled, False: Disabled
std_msgs/String ACC_Control_Board_Status  # Value [0, 7]
std_msgs/String ACC_Control_Status  # Value [0, 15]
std_msgs/String ACC_USER_CAN_ERR  # False: No Error, True: Error
std_msgs/String ACC_ERR  # False: No Error, True: Error
std_msgs/String VS  # Vehicle speed (km/h)
std_msgs/String Long_Accel  # Longitudinal acceleration, scaled 0.01 (m/s^2)
std_msgs/String Hazard_En  # True: Enabled, False: Disabled
std_msgs/String Turn_Left_En  # True: Enabled, False: Disabled
std_msgs/String Turn_Right_En  # True: Enabled, False: Disabled
std_msgs/String ACC_Veh_ERR  # False: No Error, True: Error
std_msgs/String G_SEL_DISP  # Gear selector display
std_msgs/String ACC_Alive_Cnt  # Alive counter, value [0, 255]
std_msgs/String WHEEL_SPD_FR  # Front right wheel speed, scaled 0.03125 (kph)
std_msgs/String WHEEL_SPD_FL  # Front left wheel speed, scaled 0.03125 (kph)
std_msgs/String WHEEL_SPD_RR  # Rear right wheel speed, scaled 0.03125 (kph)
std_msgs/String WHEEL_SPD_RL  # Rear left wheel speed, scaled 0.03125 (kph)
std_msgs/String LAT_ACCEL  # Lateral acceleration, scaled 0.01 (m/s^2)
std_msgs/String YAW_RATE  # Yaw rate, scaled 0.01 (deg/s)
std_msgs/String BRK_CYLINDER  # Brake cylinder pressure, scaled 0.1
std_msgs/String Long_ACCEL  # Longitudinal acceleration, scaled 0.01 (m/s^2)