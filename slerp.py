from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import pyquaternion as pyq
import numpy as np
import math as m


def slerp_pose(pose0, pose1, steps:int = 2):
    """  
    This function interpolates between two poses and returns the 
    in between poses  

    `pose0`: current pose   
    `pose1`: pose to move to   
    `steps`: how many in between steps should be executed   
        
    `return`: array with in between poses plus the last pose   DIM:(steps-1 | 6)
    """

    steps = 2 if steps < 2 else steps  # ensure a minimum of two steps

    # get in between positions
    interp_x_pos = np.linspace(pose0[0], pose1[0], steps)
    interp_y_pos = np.linspace(pose0[1], pose1[1], steps)
    interp_z_pos = np.linspace(pose0[2], pose1[2], steps)

    # combine arrays
    interp_points = np.column_stack((interp_x_pos, interp_y_pos, interp_z_pos))

    # extract rotations
    r0 = [pose0[3], pose0[4], pose0[5]]
    r1 = [pose1[3], pose1[4], pose1[5]]

    rot = R.from_euler('xyz', [r0, r1])  # define rotation

    # define start and end (nessescary for working with this library)
    (start, end) = (0, 1)

    rot_time = Slerp([start, end], rot)  # define rotation timing

    interp_times = np.linspace(start, end, steps)  # define steps

    interp_rots = rot_time(interp_times)  # create in between positions
    interp_rots = interp_rots.as_euler('xyz')  # convert to euler angles

    # combine poition and rotation arrays to one pose array
    interp_poses = np.column_stack((interp_points, interp_rots))

    interp_poses = np.delete(interp_poses, 0, 0) # remove first pose, since it is the current pose

    return interp_poses


def angle_to_turn(pose0, pose1):
    """
    This function calculates the angle between two euler poses with quaternions.
    The position gets ingnored.
    `returns`: absolute angle between poses
    """

    # extract rotations
    r0 = [pose0[3], pose0[4], pose0[5]]
    r1 = [pose1[3], pose1[4], pose1[5]]

    rot = R.from_euler('xyz', [r0, r1])  # define rotation

    rotationAsQuat = rot.as_quat()  # convert to quaterions

    # define quaternions in pyquaternion library format
    quat1 = pyq.Quaternion(rotationAsQuat[1])
    quat2 = pyq.Quaternion(rotationAsQuat[0])

    # Get the 3D difference between these two orientations
    qd = quat1.conjugate * quat2

    angleToTurn = qd.angle  # get the angle from the quaternion

    return angleToTurn


# Example-program
if __name__ == '__main__':

    # define two poses
    p1 = [0,0,0,0,0,0]
    p2 = [1,2,3,1.46,0.4,2]

    n = 5  # number of steps

    ans = slerp_pose(p1, p2, n)

    print('Slerp-Poses:')
    print(ans)

    angle =angle_to_turn(p1, p2)
    print('Angle: ', angle)

