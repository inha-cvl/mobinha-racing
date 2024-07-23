import numpy as np
import math


class ObstacleHandler:
    def __init__(self):
        self.setting_values()

    def setting_values(self):
        self.local_pose = None
        self.prev_local_pose = None
        self.current_heading = 0.0

    def update_value(self, local_pose, heading):
        if self.prev_local_pose is not None:
            self.prev_local_pose = self.local_pose
        self.local_pose = local_pose
        self.current_heading = heading
        if self.prev_local_pose is None:
            self.prev_local_pose = self.local_pose
    
    def check_dimension(self, dimension):
        if dimension[0] > 2 and dimension[1] > 1.5 :
            return True
        else:
            return False

    def object2enu(self,  obs_pose):
        if len(self.local_pose) < 1:
            return None
        rad = np.radians(self.current_heading)

        nx = math.cos(rad) * obs_pose[0] - math.sin(rad) * obs_pose[1]
        ny = math.sin(rad) * obs_pose[0] + math.cos(rad) * obs_pose[1]

        obj_x = self.local_pose[0] + nx
        obj_y = self.local_pose[1] + ny

        return obj_x, obj_y
    
    def filtering_by_lane_num(self, lane_num, fred_d):
        if -1 < fred_d < 1:
            return True
        else:
            return False
        # if lane_num == 1:
        #     if -1.45< fred_d < 4.15:
        #         return True
        #     else:
        #         return False
        # elif lane_num == 2:
        #     if -4.15 < fred_d < 4.15:
        #         return True
        #     else:
        #         return False
        # elif lane_num == 3:
        #     if -1 < fred_d < 1:
        #         return True
        #     else:
        #         return False
        # elif lane_num == 4:
        #     if -7.15 < fred_d < 1.45:
        #         return True
        #     else:
        #         return False
            
    def distance(self, x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2)

    def object2frenet(self, local_waypoints, obj_enu):
        if len(local_waypoints) > 0:  
            centerline = np.array(local_waypoints)
            point = np.array(obj_enu)

            tangents = np.gradient(centerline, axis=0)
            tangents = tangents / np.linalg.norm(tangents, axis=1)[:, np.newaxis]
            
            normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
            
            distances = np.linalg.norm(centerline - point, axis=1)
            
            closest_index = np.argmin(distances)
            closest_point = centerline[closest_index]
            tangent = tangents[closest_index]
            normal = normals[closest_index]
            
            vector_to_point = point - closest_point
            d = np.dot(vector_to_point, normal)
            s = np.sum(np.linalg.norm(np.diff(centerline[:closest_index + 1], axis=0), axis=0))
            
            return s, d
        else:
            return obj_enu[0], obj_enu[1]
    
    def get_absolute_heading(self, obs_orientation):
        siny_cosp = 2 * (obs_orientation.w * obs_orientation.z + obs_orientation.x * obs_orientation.y)
        cosy_cosp = 1 - 2 * (obs_orientation.y * obs_orientation.y + obs_orientation.z * obs_orientation.z)
        z_angle_rad = np.arctan2(siny_cosp, cosy_cosp)
        z_angle_deg = np.degrees(z_angle_rad)
        return self.current_heading - z_angle_deg

    def is_within_radius(obs_point, local_path, radius=4):
        ob_x, ob_y = obs_point
        for path_point in local_path:
            path_x, path_y = path_point
            distance = math.sqrt((ob_x - path_x) ** 2 + (ob_y - path_y) ** 2)
            if distance <= radius:
                return True
        return False