
def nmea_parser(sentence):
    parsed = sentence.split(',')
    # if parsed[0] == '$GPGGA':
    if parsed[0] == '$GNGGA':
        lat = float(parsed[2])/100.0
        lng = float(parsed[4])/100.0
        return [lat,lng]
    # elif parsed[0] == '$GPHDT':
    elif parsed[0] == '$GNHDT':
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