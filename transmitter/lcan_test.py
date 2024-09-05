import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import can
import cantools
import rospy
import asyncio
import sys
import signal

from drive_msgs.msg import RadarObjectArray, RadarObject
from std_msgs.msg import Float32MultiArray

def signal_handler(sig, frame):
    sys.exit(0)

class Visualizer:
    def __init__(self, lcan_test):
        self.lcan_test = lcan_test  # LCANTest 인스턴스 참조
        self.fig, self.ax = plt.subplots(figsize=(7, 12))  # 플롯 설정
        plt.ion()  # Interactive mode 활성화

    def init_plot(self):
        # 초기 플롯 설정
        self.ax.set_xlim(-15, 15)
        self.ax.set_ylim(0, 205)
        self.ax.set_facecolor('black')
        self.ax.grid(True, color='white')
        return []

    # 장애물 표시 함수 (매 프레임 업데이트)
    def update_plot(self):
        self.ax.cla()  # 플롯을 초기화
        self.ax.set_xlim(-15, 15)
        self.ax.set_ylim(0, 205)
        self.ax.set_facecolor('black')
        self.ax.grid(True, color='white')

        for i in range(1, 17):
            obj = getattr(self.lcan_test, f'RDR_Obj_{i:02}')
            if obj['AlvAge01'] >  1:
                x1, y1 = obj['RelPosX01'], obj['RelPosY01']
                y1 = -y1
                rect1 = patches.Rectangle((y1-1, x1-1), 1.2, 4, linewidth=1.5, edgecolor='cyan', facecolor='none')
                self.ax.add_patch(rect1)
            if obj['AlvAge02'] > 1:
                x2, y2 = obj['RelPosX02'], obj['RelPosY02']
                y2 = -y2
                rect2 = patches.Rectangle((y2-1, x2-1), 1.2, 4, linewidth=1.5, edgecolor='cyan', facecolor='none')
                self.ax.add_patch(rect2)

        # 눈금 및 축 설정
        self.ax.set_xticks(np.arange(-15, 15, 3))
        self.ax.set_yticks(np.arange(0, 206, 10))
        self.ax.set_xlabel("Lateral (m)", fontsize=9)
        self.ax.set_ylabel("Longitudinal (m)", fontsize=9)
        self.ax.tick_params(labelsize=7)
        plt.subplots_adjust(left=0.08, right=0.99, top=0.99, bottom=0.05)

        plt.draw()  # 화면을 다시 그림
        plt.pause(0.01)  # 짧은 지연 시간 대기

    # 비동기 시각화 실행
    async def run_visualization(self):
        while True:
            self.update_plot()  # 실시간 플롯 업데이트
            await asyncio.sleep(0.1)  # 0.1초마다 업데이트

class LCANTest():
    def __init__(self):
        rospy.init_node('lcan_test', anonymous=True)
        self.bus = can.ThreadSafeBus(interface='socketcan', fd=True, channel='can0', bitrate=500000)
        self.dbc = cantools.database.load_file('./RDR_Obj.dbc')
        self.setup_message_dicts()
        self.setup_decode_handlers()
        self.recording = False
        self.log_file = None

    async def read_from_can(self):
        try:
            self.start_recording()
            while not rospy.is_shutdown():
                message = await asyncio.get_event_loop().run_in_executor(None, self.bus.recv, 0.2)               
                if message:
                    can_id =  message.arbitration_id
                    data = message.data
                    self.decode_message(can_id, data)
                    if self.recording:
                        self.record_message(message)
        except Exception as e:
            rospy.logerr(f"Error in read_from_can: {e}")
        finally:
            self.stop_recording()
            self.bus.shutdown()
            
    async def replay_can(self):
        try:
            filename = "/home/kana/Documents/Dataset/Log/can_log.log"
            with open(filename, 'r') as log_file:
                next(log_file)  # 헤더 건너뛰기
                prev_timestamp = None
                
                for line in log_file:
                    parts = line.strip().split(',')
                    timestamp = float(parts[0])
                    
                    can_id = int(parts[1])
                    data = bytes.fromhex(parts[2])

                    self.decode_message(can_id, data)
                    if prev_timestamp is not None:
                        # 시간 간격 유지 (녹화된 시간에 맞춰 재생)
                        time_diff = (timestamp - prev_timestamp) / 1
                        await asyncio.sleep(time_diff)  # 기존의 time.sleep을 await asyncio.sleep으로 대체

                    prev_timestamp = timestamp
        except Exception as e:
            rospy.logerr(f"Error {e}")
    
    async def ros_publisher(self):
        try:
            await asyncio.sleep(0.5)
            cnt = 0 
            while not rospy.is_shutdown():
                await asyncio.get_event_loop().run_in_executor(None, self.publish_objects, cnt)
                cnt += 1
                await asyncio.sleep(0.05) #20hz
        except Exception as e:
            rospy.logerr(f"Error {e}")
    
    def publish_objects(self, cnt):
        self.radar_object_array_pub.publish(self.radar_object_array)
        if cnt % 1 == 0:
            self.radar_object_array = RadarObjectArray()
        self.adas_drv_pub.publish(Float32MultiArray(data=list(map(float, self.ADAS_DRV.values()))))


    def setup_decode_handlers(self):
        self.decode_handler = {
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
        self.radar_object_array = RadarObjectArray()
        self.radar_object_array_pub = rospy.Publisher('/RadarObjectArray',RadarObjectArray, queue_size=1)
        self.adas_drv_pub = rospy.Publisher('/ADAS_DRV', Float32MultiArray,queue_size=1)

    def decode_message(self, id, data):

        if id in self.decode_handler.keys():
            getter_dict = self.decode_handler.get(id)
            decoded_message = self.dbc.decode_message(id, data)
            if id == 81:
                getter_dict.update({key: decoded_message.get(key) for key in getter_dict.keys()})
            else:
              
                for getter_key in getter_dict.keys():
                    base_key_getter = getter_key[:-2]  # '01', '02' 등의 숫자 제거한 부분
                    suffix_getter = getter_key[-2:]  # '01' 또는 '02' 숫자 부분

                    for decoded_key in decoded_message.keys():
                        base_key_decoded = decoded_key[:-2]  # '01', '02', '03', '04', '05', '06' 등을 제거한 부분
                        suffix_decoded = decoded_key[-2:]  # 숫자 부분 ('01', '02', ..., '31', '32')

                        if base_key_getter == base_key_decoded:
                            if int(suffix_decoded) % 2 == 1 and suffix_getter == '01':
                                getter_dict[getter_key] = decoded_message[decoded_key]
                            elif int(suffix_decoded) % 2 == 0 and suffix_getter == '02':
                                getter_dict[getter_key] = decoded_message[decoded_key]

                if getter_dict['AlvAge01'] > 1 and (-50<getter_dict['RelPosY01']<50):
                    radar_object = RadarObject()
                    radar_object.alvAge.data = int(getter_dict['AlvAge01'])
                    radar_object.coastAge.data = int(getter_dict['CoastAge01'])
                    radar_object.trkSta.data = int(getter_dict['TrkSta01'])
                    radar_object.mvngFlag.data = int(getter_dict['MvngFlag01'])
                    radar_object.qualLvl.data = int(getter_dict['QualLvl01'])
                    radar_object.relPosX.data = float(getter_dict['RelPosX01'])
                    radar_object.relPosY.data = float(getter_dict['RelPosY01'])
                    radar_object.relVelX.data = float(getter_dict['RelVelX01'])
                    radar_object.relVelY.data = float(getter_dict['RelVelY01'])
                    radar_object.relAccel.data = float(getter_dict['RelAccelX01'])
                    self.radar_object_array.radarObjects.append(radar_object)
                if getter_dict['AlvAge02'] > 1 and (-50<getter_dict['RelPosY02']<50):
                    radar_object = RadarObject()
                    radar_object.alvAge.data = int(getter_dict['AlvAge02'])
                    radar_object.coastAge.data = int(getter_dict['CoastAge02'])
                    radar_object.trkSta.data = int(getter_dict['TrkSta02'])
                    radar_object.mvngFlag.data = int(getter_dict['MvngFlag02'])
                    radar_object.qualLvl.data = int(getter_dict['QualLvl02'])
                    radar_object.relPosX.data = float(getter_dict['RelPosX02'])
                    radar_object.relPosY.data = float(getter_dict['RelPosY02'])
                    radar_object.relVelX.data = float(getter_dict['RelVelX02'])
                    radar_object.relVelY.data = float(getter_dict['RelVelY02'])
                    radar_object.relAccel.data = float(getter_dict['RelAccelX02'])
                    self.radar_object_array.radarObjects.append(radar_object)

    def record_message(self, message):
        """CAN 메시지를 로그 파일에 기록하는 함수"""
        timestamp = message.timestamp
        can_id = message.arbitration_id
        data = message.data.hex()

        if self.log_file:
            self.log_file.write(f"{timestamp},{can_id},{data}\n")  # 로그 파일에 CAN 메시지 기록

    def start_recording(self):
        """녹화를 시작하는 함수"""
        self.recording = True
        self.log_file = open('./can_log.log', 'w')  # 로그 파일 열기
        self.log_file.write("Timestamp,ID,Data\n")  # 로그 헤더 작성
        print("Recording started.")

    def stop_recording(self):
        """녹화를 중지하는 함수"""
        self.recording = False
        if self.log_file:
            self.log_file.close()  # 로그 파일 닫기
            self.log_file = None
            print("Recording stopped and saved.")

    def setup_message_dicts(self):
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

    def run(self):
        loop = asyncio.get_event_loop()
        read_task = loop.create_task(self.read_from_can())  # CAN 메시지를 읽는 작업
        #replay_task = loop.create_task(self.replay_can())
        #visualize_task = loop.create_task(self.lcan_visualizer.run_visualization())  # 비동기 시각화 작업
        ros_task = loop.create_task(self.ros_publisher())
        loop.run_forever()

    def set_visualizer(self, visualizer):
        self.lcan_visualizer = visualizer


def main():
    signal.signal(signal.SIGINT, signal_handler)
    lcan_test = LCANTest()
    visualizer = Visualizer(lcan_test)
    lcan_test.set_visualizer(visualizer)
    lcan_test.run()



if __name__ == "__main__":
    main()
