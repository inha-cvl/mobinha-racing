
def gps_to_decimal_degrees(nmea_pos):
    if len(nmea_pos) > 5:
        digit_count = 2 if nmea_pos[4] == '.' else 3
        integer_part = nmea_pos[:digit_count]
        decimal_part = nmea_pos[digit_count:]
        v = float(integer_part) + float(decimal_part) / 60.0
        return v
    return 0.0
   
def nmea_parser(sentence):
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
            return [lat,lng]
    
    elif parsed[0] == '$GNHDT':
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