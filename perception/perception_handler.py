import math

def calculate_heading(rel_vel_x, rel_vel_y):
    heading_radians = math.atan2(rel_vel_x, -rel_vel_y)
    heading_degrees = (math.degrees(heading_radians)+90)
    return heading_degrees

def calculate_velocity(rel_vel_x, rel_vel_y):
    relative_speed = math.sqrt(rel_vel_x**2 + rel_vel_y**2)
    return relative_speed

# 두 점 사이의 거리를 계산하는 함수
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# 0.5 안에 포함되는지 확인하며 군집화하는 함수
def cluster_points(data, distance_threshold=0.7):
    clusters = []  # 군집화된 결과를 저장할 리스트
    visited = [False] * len(data)  # 방문한 포인트를 체크할 리스트

    for i in range(len(data)):
        if visited[i]:
            continue
        
        x1 = data[i][0]
        y1 = data[i][1]
        cluster = [data[i]]  # 새로운 군집 시작

        # i 번째와 다른 점들을 비교
        for j in range(i+1, len(data)):
            if visited[j]:
                continue

            x2 = data[j][0]
            y2 = data[j][1]

            # 거리가 0.5 이하인 경우 군집에 추가
            if calculate_distance(x1, y1, x2, y2) <= distance_threshold:
                cluster.append(data[j])
                visited[j] = True
        
        # 나이(age)가 가장 높은 포인트를 기준으로 heading과 velocity를 설정
        if len(cluster) > 1:
            max_age_point = max(cluster, key=lambda x: x[4])
            heading = calculate_heading(max_age_point[2], max_age_point[3])
            velocity = calculate_velocity(max_age_point[2], max_age_point[3])
            clusters.append([x1, y1,  heading, velocity, float(max_age_point[4])])
        else:
            heading = calculate_heading(data[i][2], data[i][3])
            velocity = calculate_velocity(data[i][2], data[i][3])
            clusters.append([data[i][0], data[i][1], heading, velocity, float(data[i][4])])  # 혼자 남은 포인트는 그대로 추가
    
    return clusters

def filtering_by_spd(data, spd):
    filtered_list = []
    for obj in data:
        if obj[3] - spd == 0:
            pass
        else:
            filtered_list.append(obj)
    return filtered_list