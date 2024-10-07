import rospy

from std_msgs.msg import Header, Float32MultiArray
from drive_msgs.msg import *



class ROSHandler():
    def __init__(self, TH):
        
        rospy.init_node('transmitter', anonymous=False)
        self.decode_handler0 = TH.decode_handler0
        self.decode_handler1 = TH.decode_handler1
        self.decode_handler2 = TH.decode_handler2
        self.encode_handler = TH.encode_handler
        self.ADAS_DRV = TH.ADAS_DRV
        self.set_protocol()
        self.set_messages()

        self.alive_cnt = -1

    def set_protocol(self):
        self.can_output_pub = rospy.Publisher('/CANOutput', CANOutput, queue_size=1)
        rospy.Subscriber('/CANInput', CANInput, self.can_input_cb)
        self.ccan_output_pub = rospy.Publisher('/CCANOutput', CCANOutput, queue_size=1)
        self.radar_object_array_pub = rospy.Publisher('/RadarObjectArray',RadarObjectArray, queue_size=1)
        self.adas_drv_pub = rospy.Publisher('/ADAS_DRV', Float32MultiArray,queue_size=1)


    def set_messages(self):
        self.can_output = CANOutput()
        for values in self.decode_handler0.values():
            for key, value in values.items():
                getattr(self.can_output, key).data = value

        self.can_input = CANInput()
        for values in self.encode_handler.values():
            for key, value in values.items():
                getattr(self.can_input, key).data = value
        
        self.ccan_output = CCANOutput()
        for values in self.decode_handler1.values():
            for key, value in values.items():
                getattr(self.ccan_output, key).data = value
        
        self.radar_object_array = RadarObjectArray()
        
    def update_can_output(self, message_dict):
        for key, value in message_dict.items():
            getattr(self.can_output, key).data = str(value)
    
    def update_ccan_output(self, message_dict):
        for key, value in message_dict.items():
            getattr(self.ccan_output, key).data = str(value)
    
    def update_radar_output(self, type, message_dict):
        if type == 0:
            return 
        
        if message_dict['AlvAge01'] > 1 :#and (-80<message_dict['RelPosY01']<80):
            radar_object = RadarObject()
            radar_object.alvAge.data = int(message_dict['AlvAge01'])
            radar_object.coastAge.data = int(message_dict['CoastAge01'])
            radar_object.trkSta.data = int(message_dict['TrkSta01'])
            radar_object.mvngFlag.data = int(message_dict['MvngFlag01'])
            radar_object.qualLvl.data = int(message_dict['QualLvl01'])
            radar_object.relPosX.data = float(message_dict['RelPosX01'])
            radar_object.relPosY.data = float(message_dict['RelPosY01'])
            radar_object.relVelX.data = float(message_dict['RelVelX01'])
            radar_object.relVelY.data = float(message_dict['RelVelY01'])
            radar_object.relAccel.data = float(message_dict['RelAccelX01'])
            self.radar_object_array.radarObjects.append(radar_object)
        if message_dict['AlvAge02'] > 1 :#and (-80<message_dict['RelPosY02']<80):
            radar_object = RadarObject()
            radar_object.alvAge.data = int(message_dict['AlvAge02'])
            radar_object.coastAge.data = int(message_dict['CoastAge02'])
            radar_object.trkSta.data = int(message_dict['TrkSta02'])
            radar_object.mvngFlag.data = int(message_dict['MvngFlag02'])
            radar_object.qualLvl.data = int(message_dict['QualLvl02'])
            radar_object.relPosX.data = float(message_dict['RelPosX02'])
            radar_object.relPosY.data = float(message_dict['RelPosY02'])
            radar_object.relVelX.data = float(message_dict['RelVelX02'])
            radar_object.relVelY.data = float(message_dict['RelVelY02'])
            radar_object.relAccel.data = float(message_dict['RelAccelX02'])
            self.radar_object_array.radarObjects.append(radar_object)
    
    def update_can_inputs(self):
        self.update_alive_cnt()
        dicts = []
        for values in self.encode_handler.values():
            for key in values.keys():
                if key == 'Aliv_Cnt':
                    values[key] = self.alive_cnt
                else:
                    values[key]=getattr(self.can_input, key).data
            dicts.append(values)
        return dicts

    def update_alive_cnt(self):
        self.alive_cnt += 1
        if self.alive_cnt >= 256:
            self.alive_cnt = 0

    def send_can_output(self):
        self.can_output.header = Header()
        self.can_output.header.stamp = rospy.Time.now()
        self.can_output_pub.publish(self.can_output)
    
    def send_ccan_output(self):
        self.ccan_output_pub.publish(self.ccan_output)
    
    def send_radar_output(self, cnt):
        self.radar_object_array.header = Header()
        self.radar_object_array.header.stamp = rospy.Time.now()
        self.radar_object_array_pub.publish(self.radar_object_array)
        if cnt % 3 == 0:
            self.radar_object_array = RadarObjectArray()
        self.adas_drv_pub.publish(Float32MultiArray(data=list(map(float, self.ADAS_DRV.values()))))


    def can_input_cb(self, msg):
        self.can_input = msg