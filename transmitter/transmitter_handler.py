
import cantools
import can

class TransmitterHandler:
    def __init__(self):
        self.dbc = cantools.database.load_file('./cn7.dbc')
        
        self.setup_message_dicts()
        self.setup_decode_handlers()
        self.setup_encode_handler()

    def setup_encode_handler(self):
        self.encode_handler = {
            0x156: self.EAIT_Control_01,
            0x157: self.EAIT_Control_02,
            # 0x126: self.EAIT_REPAIR_TEMP
        }

        self.encode_dbc = {
            0x156: 'EAIT_Control_01',
            0x157: 'EAIT_Control_02'
        }

    def setup_decode_handlers(self):
        self.decode_handler = {
            0x710: self.EAIT_INFO_EPS,
            0x711: self.EAIT_INFO_ACC,
            0x712: self.EAIT_INFO_SPD,
            0x713: self.EAIT_INFO_IMU
            # 0x124: self.EAIT_SIG_TEMP
        }
    
    def decode_message(self, message):
        _id = message.arbitration_id
        if _id in self.decode_handler.keys():
            getter_dict = self.decode_handler.get(_id)
            decoded_message = self.dbc.decode_message(_id, message.data)
            getter_dict.update({key: decoded_message.get(key) for key in getter_dict.keys()})
            return getter_dict
    
    def encode_message(self, dicts):
        can_messages = []
        for i, (key,value) in enumerate(self.encode_dbc.items()):
            message = self.dbc.encode_message(value, dicts[i])
            can_message = can.Message(arbitration_id=key, data=message, is_extended_id=False)
            can_messages.append(can_message)
        return can_messages
    
    def setup_message_dicts(self):

        self.EAIT_Control_01 = {
            'EPS_En': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'EPS_Override_Ignore': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'EPS_Speed': 10,  # 1.0 * (value) [0,250]
            'ACC_En': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'AEB_En': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'Turn_Signal': 0x00,  # 0x00: Off, 0x01: Emergency, 0x02: Right, 0x03: Left
            'AEB_decel_value': 0,  # 0.01 * (value) [0.0,1.0]
            'Aliv_Cnt': 0  # 1.0 * (value) [0,255]
        }

        self.EAIT_Control_02 = {
            'EPS_Cmd': 0,  # 0.1 * (value) [-500, 500] "deg"
            'ACC_Cmd': 0  # 0.01 * (value + 10.23) [-3, 1] "m/s^2"
        }

        self.EAIT_INFO_EPS = {
            'EPS_En_Status': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'EPS_Control_Board_Status': 0,  # value [0, 2]
            'EPS_Control_Status': 0,  # value [0, 15]
            'EPS_USER_CAN_ERR': 0x00,  # 0x00: No Error, 0x01: Error
            'EPS_ERR': 0x00,  # 0x00: No Error, 0x01: Error
            'EPS_Veh_CAN_ERR': 0x00,  # 0x00: No Error, 0x01: Error
            'EPS_SAS_ERR': 0x00,  # 0x00: No Error, 0x01: Error
            'Override_Ignore_Status': 0x00,  # 0x00: Not Ignored, 0x01: Ignored
            'Override_Status': 0x00,  # 0x00: No Override, 0x01: Override
            'StrAng': 0,  # 0.1 * (value) [-500, 500] "deg"
            'Str_Drv_Tq': 0,  # 0.01 * (value + 20.48) [-20.48, 20.47] ""
            'Str_Out_Tq': 0,  # 0.1 * (value + 204.8) [-204.8, 204.7] ""
            'EPS_Alive_Cnt': 0  # value [0, 255]
        }

        self.EAIT_INFO_ACC = {
            'ACC_En_Status': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'ACC_Control_Board_Status': 0,  # value [0, 7]
            'ACC_Control_Status': 0,  # value [0, 15]
            'ACC_USER_CAN_ERR': 0x00,  # 0x00: No Error, 0x01: Error
            'ACC_ERR': 0x00,  # 0x00: No Error, 0x01: Error
            'VS': 0,  # value [0, 255] "km/h"
            'Long_Accel': 0,  # 0.01 * (value + 10.23) [-10.23, 10.24] "m/s^2"
            'Hazard_En': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'Turn_Left_En': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'Turn_Right_En': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'ACC_Veh_ERR': 0x00,  # 0x00: No Error, 0x01: Error
            'G_SEL_DISP': 0,  # value [0, 15]
            'ACC_Alive_Cnt': 0  # value [0, 255]
        }

        self.EAIT_INFO_SPD = {
            'WHEEL_SPD_FR': 0,  # 0.03125 * (value) [0, 511.96875] "kph"
            'WHEEL_SPD_FL': 0,  # 0.03125 * (value) [0, 511.96875] "kph"
            'WHEEL_SPD_RR': 0,  # 0.03125 * (value) [0, 511.96875] "kph"
            'WHEEL_SPD_RL': 0  # 0.03125 * (value) [0, 511.96875] "kph"
        }

        self.EAIT_INFO_IMU = {
            'LAT_ACCEL': 0,  # 0.01 * (value + 10.23) [-10.23, 10.23] "m/s^2"
            'YAW_RATE': 0,  # 0.01 * (value + 40.95) [-40.95, 40.94] "deg/s"
            'BRK_CYLINDER': 0,  # 0.1 * (value) [0, 409.4] ""
            'Long_ACCEL': 0  # 0.01 * (value + 10.23) [-10.23, 10.23] "m/s^2"
        }

        # self.EAIT_SIG_TEMP = { # temporary naming due to lack of dbc
        #     'SIG_GO': 0x00, # 0x00: Disabled, 0x01: Enabled
        #     'SIG_STOP': 0x00, # 0x00: Disabled, 0x01: Enabled
        #     'SIG_PIT_STOP': 0x00, # 0x00: Disabled, 0x01: Enabled
        #     'SIG_SLOW_ON': 0x00, # 0x00: Disabled, 0x01: Enabled
        #     'SIG_SLOW_OFF': 0x00, # 0x00: Disabled, 0x01: Enabled
        #     'reserved': 0x00 # reserved for future function expansion
        # }

        # self.EAIT_REPAIR_TEMP = { # temporary naming due to lack of dbc
        #     'SIG_REPAIR': 0xFF # 0xFF: Repair Enable, 0xFF > Value: Repair Disable
        # }