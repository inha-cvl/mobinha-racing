import tf
import math
import numpy as np

def gps_to_decimal_degrees(nmea_pos):
    if len(nmea_pos) > 5:
        digit_count = 2 if nmea_pos[4] == '.' else 3
        integer_part = nmea_pos[:digit_count]
        decimal_part = nmea_pos[digit_count:]
        v = float(integer_part) + float(decimal_part) / 60.0
        return v
    return 0.0


def calculate_heading(lat1, lon1, lat2, lon2):
  
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    dLon = lon2 - lon1
    
    x = math.sin(dLon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dLon))
    
    initial_bearing = math.atan2(x, y)
    
    initial_bearing = math.degrees(initial_bearing)
    
    compass_bearing = (initial_bearing + 360) % 360
    heading = ((-1*(compass_bearing)+450)%360)+180
    
    return heading



def nmea_parser(lat0, lng0, sentence):
    parsed = sentence.split(',')
    if parsed[0] == '$GNGGA':
        lat_gga = parsed[2]
        lng_gga = parsed[4]
        if len(lat_gga) < 10 or len(lng_gga) < 10:
            return None
        else:
            if lat_gga[-1] == 'N':
                lat_gga = str(lat_gga[0:-1])
            if lng_gga[-1] == 'E':
                lng_gga = str(lng_gga[0:-1])
            
            lat = gps_to_decimal_degrees(lat_gga)
            lng = gps_to_decimal_degrees(lng_gga)

            if lat0 == 0 or lng0 == 0:
                return [lat, lng, 0]
            else:
                heading = calculate_heading(lat0, lng0, lat, lng)
                return [lat,lng, heading]
    
    elif parsed[0] == '$GNTHS':
        heading =parsed[1]
        if 'T' in heading:
            heading = heading[0:-4]
        heading = (-1*(float(heading)+450)%360)+180
        return [float(heading)]
    else:
        return None


def sim_nmea_parser(sentence):
    parsed = sentence.split(',')
    if parsed[0] == '$GPGGA':
        lat = float(parsed[2])/100.0
        lng = float(parsed[4])/100.0
        return [lat,lng]
    elif parsed[0] == '$GPHDT':
        heading = float(parsed[1])
        return [heading]

def check_error(a,b, bound):
    if a == 0:
        return False
    error_boundary = bound
    error = abs(a-b)
    return False if error<error_boundary else True
    
def match_heading(x, y, z, w):
    _, _, yaw = tf.transformations.euler_from_quaternion([x,y,z,w])
    yaw = (-1*(float(math.degrees(yaw))+450)%360)+180
    return yaw
    
def calc_wheel_velocity(vRR, vRL):
    return (float(vRR) + float(vRL))/7.2    

def mode_checker(eps_status, acc_status):
    status_to_mode = {
        ('Ready', 'Ready'): 0, # Off
        ('All_On', 'All_On'): 1, # On
        ('EPS_On', 'EPS_On'): 2, # Steering Only
        ('ACC_On', 'ACC_On'): 3 # Acce/Brake Only
    }
    mode = status_to_mode.get((eps_status, acc_status), 0)  # 기본값으로 0을 사용

    return mode

def turn_signal_checker(turn_left_en, turn_right_en):
    if turn_left_en == 'On':
        return 1
    elif turn_right_en == 'On':
        return 2
    else:
        return 0

def convert_local_to_enu(ego_pose, ego_heading, obj_pose):
    if len(ego_pose) < 1:
        return None
    else:
        rad = np.radians(ego_heading)
        
        nx = math.cos(rad) * obj_pose[0] - math.sin(rad) * obj_pose[1]
        ny = math.sin(rad) * obj_pose[0] + math.cos(rad) * obj_pose[1]
        
        x = ego_pose[0]+nx
        y = ego_pose[1]+ny
        
        return x,y

def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

#TODO # lap_number
def check_lap_count(cnt, local_pos, reference_point, radius=20, lap_flag=False):
    if local_pos is None:
        return cnt, lap_flag
    distance = math.sqrt((local_pos[0] - reference_point[0])**2 + (local_pos[1] - reference_point[1])**2)
    if distance < radius and lap_flag:
        cnt += 1
        lap_flag = False
    elif distance > radius:
        lap_flag = True
    return cnt, lap_flag

