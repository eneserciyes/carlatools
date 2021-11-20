import numpy as np
import math
import carla

import gnss

def gnss_to_carla_coord(lat, lon, alt):
    x, y, z = gnss.geodetic_to_ecef(lat, lon, alt)
    carla_x, carla_y, carla_z = gnss.ecef_to_enu(x, y, z, 0.0, 0.0, 0.0)
    return carla_x, -carla_y, carla_z


def mat_to_trans(M):
    """
    Convert 4x4 transformation matrix to carla.Transform object
    """
    sy = math.sqrt(M[0,0]**2 + M[1,0]**2)
    if not (sy < 1e-6):
        pitch = math.atan2(M[2,1], M[2,2])
        roll = math.atan2(-M[2,0], sy)
        yaw = math.atan2(M[1,0], M[0,0])
    else:
        pitch = math.atan2(-M[1,2], M[1,1])
        roll = math.atan2(-M[2,0], sy)
        yaw = 0
    T = carla.Transform(carla.Location(M[0,3], M[1,3], M[2,3]), carla.Rotation(pitch, roll, yaw))
    return T


def relative_transform(t_origin, t_target):
    """
    Get the target transformation in a given origin frame

    Params:
     - t_origin: Origin frame transformation (carla.Transform)
     - t_target: Target transformation (carla.Transform)
    
    Returns:
     - t_target in t_origin frame as carla.Transform object 
    """
    new_mat = np.dot(np.asarray(t_origin.get_inverse_matrix()), np.asarray(t_target.get_matrix()))
    return mat_to_trans(new_mat)



def _numpy(carla_vector, normalize=False):
    result = np.float32([carla_vector.x, carla_vector.y])

    if normalize:
        return result / (np.linalg.norm(result) + 1e-4)

    return result

def _orientation(yaw):
    return np.float32([np.cos(np.radians(yaw)), np.sin(np.radians(yaw))])
