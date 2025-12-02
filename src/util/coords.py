import numpy as np

def coords_to_ego(bbox_worldcord, ego_vehicle_worldcord):
    # bbox_worldcord and ego_vehicle_worldcord are in format [x,y,z, roll, pitch, yaw] (degrees)
    
    def rotation_matrix(roll, pitch, yaw):
        roll = roll * np.pi / 180
        pitch = pitch * np.pi / 180
        yaw = yaw * np.pi / 180
        
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
        R = R_z @ R_y @ R_x
        return R
    
    
    # Extract positions and orientations
    bbox_pos = np.array(bbox_worldcord[0:3])
    bbox_rot = bbox_worldcord[3:6]
    ego_pos = np.array(ego_vehicle_worldcord[0:3])
    ego_rot = ego_vehicle_worldcord[3:6]

    # Compute rotation matrices
    R_ego = rotation_matrix(ego_rot[0], ego_rot[1], ego_rot[2])
    R_ego_inv = R_ego.T  # Inverse of rotation matrix is its transpose for rotation matrices
    R_bbox = rotation_matrix(bbox_rot[0], bbox_rot[1], bbox_rot[2])

    bbox_ego_coords = R_ego_inv @ (bbox_pos - ego_pos)
    bbox_ego_rotation = R_ego_inv @ R_bbox
    # Convert rotation matrix back to Euler angles
    sy = np.sqrt(bbox_ego_rotation[0,0] ** 2 + bbox_ego_rotation[1,0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(bbox_ego_rotation[2,1], bbox_ego_rotation[2,2])
        y = np.arctan2(-bbox_ego_rotation[2,0], sy)
        z = np.arctan2(bbox_ego_rotation[1,0], bbox_ego_rotation[0,0])
    else:
        x = np.arctan2(-bbox_ego_rotation[1,2], bbox_ego_rotation[1,1])
        y = np.arctan2(-bbox_ego_rotation[2,0], sy)
        z = 0   
    bbox_ego_rotation = np.array([x, y, z])
    
    bbox_ego_rotation = bbox_ego_rotation * 180 / np.pi

    return bbox_ego_coords, bbox_ego_rotation

def rotation_matrix(roll, pitch, yaw):
    roll = np.deg2rad(roll)
    pitch = np.deg2rad(pitch)
    yaw = np.deg2rad(yaw)

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.stack([
        np.stack([np.ones_like(roll), 0*roll, 0*roll], axis=1),
        np.stack([0*roll, cr, -sr], axis=1),
        np.stack([0*roll, sr, cr], axis=1)
    ], axis=1)
    
    Ry = np.stack([
        np.stack([cp, 0*pitch, sp], axis=1),
        np.stack([0*pitch, np.ones_like(pitch), 0*pitch], axis=1),
        np.stack([-sp, 0*pitch, cp], axis=1)
    ], axis=1)
    
    Rz = np.stack([
        np.stack([cy, -sy, 0*yaw], axis=1),
        np.stack([sy, cy, 0*yaw], axis=1),
        np.stack([0*yaw, 0*yaw, np.ones_like(yaw)], axis=1)
    ], axis=1)
    
    return Rz @ Ry @ Rx 

def coords_to_ego_vectorized(bbox_worldcord, ego_vehicle_worldcord):
    """bbox_worldcord should be of (N, 6)
    """
    # Convert bbox from world coordinates to ego vehicle coordinates
    bbox_worldcord = np.asarray(bbox_worldcord)
    ego_vehicle_worldcord = np.asarray(ego_vehicle_worldcord)
    
    Re = rotation_matrix(
        np.array([ego_vehicle_worldcord[3]]),
        np.array([ego_vehicle_worldcord[4]]),
        np.array([ego_vehicle_worldcord[5]])
    )[0]

    Rw = rotation_matrix(
        bbox_worldcord[:, 3], 
        bbox_worldcord[:, 4], 
        bbox_worldcord[:, 5]
    )

    Rbe = Re.T @ Rw
    
    roll = np.arctan2(Rbe[:,2,1], Rbe[:,2,2])
    pitch = np.arcsin(-Rbe[:,2,0])
    yaw = np.arctan2(Rbe[:,1,0], Rbe[:,0,0])

    bbox_ego_coords = (Re.T @ (bbox_worldcord[:, :3] - ego_vehicle_worldcord[:3]).T).T

    bbox_ego_rotation = np.stack([roll, pitch, yaw], axis=-1)
    bbox_ego_rotation = np.rad2deg(bbox_ego_rotation)

    return bbox_ego_coords, bbox_ego_rotation