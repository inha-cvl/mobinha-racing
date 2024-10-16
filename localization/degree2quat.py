import numpy as np
from scipy.spatial.transform import Rotation as R

def heading_to_quaternion(heading_deg: float) -> np.ndarray:

    yaw_rad = np.deg2rad(heading_deg)

    # Euler angles -> Quaternion 변환 (yaw만 사용)
    rotation = R.from_euler('z', yaw_rad)
    quaternion = rotation.as_quat() 

    return quaternion

