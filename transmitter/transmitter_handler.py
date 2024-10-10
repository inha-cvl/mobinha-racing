
import cantools
import can
import cantools.database

class TransmitterHandler:
    def __init__(self):
        self.dbc0 = cantools.database.load_file('./cn7.dbc')
        self.dbc1 = cantools.database.load_file('./hyundai_ccan.dbc')
        self.dbc2 = cantools.database.load_file('./RDR_Obj.dbc')
        self.setup_message_dicts()
        self.setup_decode_handlers()
        self.setup_encode_handler()

    def setup_encode_handler(self):
        self.encode_handler = {
            0x156: self.EAIT_Control_01,
            0x157: self.EAIT_Control_02,
            0x126: self.KIAPI_3
        }

        self.encode_dbc = {
            0x156: 'EAIT_Control_01',
            0x157: 'EAIT_Control_02'
        }

    def setup_decode_handlers(self):
        self.decode_handler0 = {
            0x710: self.EAIT_INFO_EPS,
            0x711: self.EAIT_INFO_ACC,
            0x712: self.EAIT_INFO_SPD,
            0x713: self.EAIT_INFO_IMU,
            0x124: self.KIAPI_1
        }

        self.decode_handler1 = {
            1419: self.LCA11,
            # 832: self.LKAS11
        }

        self.decode_handler2 = {
            0x210: self.RDR_Obj_01,
            0x211: self.RDR_Obj_02,
            0x212: self.RDR_Obj_03,
            0x213: self.RDR_Obj_04,
            0x214: self.RDR_Obj_05,
            0x215: self.RDR_Obj_06,
            0x216: self.RDR_Obj_07,
            0x217: self.RDR_Obj_08,
            0x218: self.RDR_Obj_09,
            0x219: self.RDR_Obj_10,
            0x21A: self.RDR_Obj_11,
            0x21B: self.RDR_Obj_12,
            0x21C: self.RDR_Obj_13,
            0x21D: self.RDR_Obj_14,
            0x21E: self.RDR_Obj_15,
            0x21F: self.RDR_Obj_16,
            0x51: self.ADAS_DRV
        }


    
    def decode_message0(self, message):
        _id = message.arbitration_id
        if _id in self.decode_handler0.keys():
            getter_dict = self.decode_handler0.get(_id)
            decoded_message = self.dbc0.decode_message(_id, message.data)
            getter_dict.update({key: decoded_message.get(key) for key in getter_dict.keys()})
            return getter_dict
    
    def encode_message(self, dicts):
        can_messages = []
        for i, (key,value) in enumerate(self.encode_dbc.items()):
            message = self.dbc0.encode_message(value, dicts[i])
            can_message = can.Message(arbitration_id=key, data=message, is_extended_id=False)
            can_messages.append(can_message)
        return can_messages

    def decode_message1(self, message):
        _id = message.arbitration_id
        if _id in self.decode_handler1.keys():
            getter_dict = self.decode_handler1.get(_id)
            decoded_message = self.dbc1.decode_message(_id, message.data)
            getter_dict.update({key: decoded_message.get(key) for key in getter_dict.keys()})
            return getter_dict

    def decode_message2(self, message):
        _id = message.arbitration_id
        if _id in self.decode_handler2.keys():
            getter_dict = self.decode_handler2.get(_id)
            decoded_message = self.dbc2.decode_message(_id, message.data)
            if _id == 81:
                getter_dict.update({key: decoded_message.get(key) for key in getter_dict.keys()})
                type = 0
            else:
                type = 1
                for getter_key in getter_dict.keys():
                    base_key_getter = getter_key[:-2] 
                    suffix_getter = getter_key[-2:]

                    for decoded_key in decoded_message.keys():
                        base_key_decoded = decoded_key[:-2] 
                        suffix_decoded = decoded_key[-2:] 
                        if base_key_getter == base_key_decoded:
                            if int(suffix_decoded) % 2 == 1 and suffix_getter == '01':
                                getter_dict[getter_key] = decoded_message[decoded_key]
                            elif int(suffix_decoded) % 2 == 0 and suffix_getter == '02':
                                getter_dict[getter_key] = decoded_message[decoded_key]

            return type, getter_dict
        else:
            return None, None
        
    def setup_message_dicts(self):

        self.EAIT_Control_01 = {
            'EPS_En': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'EPS_Override_Ignore': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'EPS_Speed': 10,  # 1.0 * (value) [0,250]
            'ACC_En': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'AEB_En': 0x00,  # 0x00: Disabled, 0x01: Enabled
            'Turn_Signal': 0x00,  # 0x00: Off, 0x01: Emergency, 0x02: Left, 0x03: Right
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

        self.KIAPI_1 = { # temporary naming due to lack of dbc
            'SIG_GO': 'Off', # 0x00: Disabled, 0x01: Enabled
            'SIG_STOP': 'Off', # 0x00: Disabled, 0x01: Enabled
            'SIG_PIT_STOP': 'Off', # 0x00: Disabled, 0x01: Enabled
            'SIG_SLOW_ON': 'Off', # 0x00: Disabled, 0x01: Enabled
            'SIG_SLOW_OFF': 'Off', # 0x00: Disabled, 0x01: Enabled
            'reserved': 'Off' # reserved for future function expansion
        }

        self.KIAPI_3 = { # temporary naming due to lack of dbc
            'SIG_REPAIR': 0xFF # 0xFF: Repair Enable, 0xFF > Value: Repair Disable
        }

        self.LCA11 = {
            'CF_Lca_Stat': "",
            'CF_Rcta_Stat':"",
            'CF_Lca_IndLeft' : "",
            'CF_Lca_IndRight': "",
            'CF_RCTA_IndLeft' : "",
            'CF_RCTA_IndRight' :"",

        }

        self.SCC12 = {
            'aReqMax': "",
            'aReqMin':"",
        }

        self.ADAS_DRV = {
            'CRC01':0,
            'AlvCnt01':0,
            'Radar_RadiCmdSta':0,
            'YawRateVal':0,
            'SASAngleVal':0,
            'EstVehSpdVal':0,
            'EstRadVal':0,
            'CIPVRadarIDVal':0,
            'CIPSRadarIDVal':0,
            'StrTqSeldSta':0,
            'FcaCIPVRadarIDVal':0,
            'WhlSpdCalcVal':0,
            'VehTgtRelDistVal':0,
            'VehTgtRelSpdVal':0,
            'PedTgtRelDistVal':0,
            'PedTgtRelSpdVal':0,
            'JTTgtRelDistVal':0,
            'JTTgtRelSpdVal':0,
            'EngRunSta':0
        }

        base_RDR_Obj = {
            'RefObjID01': 1,
            'AlvAge01': 0,
            'TrkSta01': 0,
            'MvngFlag01': 0,
            'QualLvl01':0,
            'CoastAge01':0,
            'RelPosX01': 0.05,
            'RelPosY01': 0.05,
            'RelVelX01': 0.01,
            'RelVelY01': 0.01,
            'RelAccelX01': 0.05,
            'RefObjID02': 1,
            'AlvAge02': 0,
            'TrkSta02': 0,
            'MvngFlag02': 0,
            'QualLvl02':0,
            'CoastAge02':0,
            'RelPosX02': 0.05,
            'RelPosY02': 0.05,
            'RelVelX02': 0.01,
            'RelVelY02': 0.01,
            'RelAccelX02': 0.05
        }
        
        # 반복문을 통해 RDR_Obj_01 ~ RDR_Obj_16까지 설정
        for i in range(1, 17):  # 1부터 16까지
            setattr(self, f'RDR_Obj_{i:02}', base_RDR_Obj.copy())
        
        
