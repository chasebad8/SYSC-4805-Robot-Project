
import sim
import math
MAX_DISTANCE = 1000.0  # if no detection, return this.


def get_distance(sim_handle, sensor_handle):
    res, detected, det_point = sim.simxReadProximitySensor(sim_handle, sensor_handle, sim.simx_opmode_buffer)[0:3]
    if res == sim.simx_return_ok and detected:
        print(det_point)
        return math.sqrt(det_point[0] * det_point[0] + det_point[1] * det_point[1] + det_point[2] * det_point[2])
    else:
        return MAX_DISTANCE  # This should actually be an error.


# I think this one is the best, so much time wasted lol
def flat_distance(sim_handle, sensor_handle):
    res, detected, det_point = sim.simxReadProximitySensor(sim_handle, sensor_handle, sim.simx_opmode_buffer)[0:3]
    if res == sim.simx_return_ok and detected:
        return det_point[2]
    else:
        return MAX_DISTANCE  # This should actually be an error.


def get_normal_distance(sim_handle, sensor_handle):
    res, detected, det_point = sim.simxReadProximitySensor(sim_handle, sensor_handle, sim.simx_opmode_buffer)[0:3]
    if res == sim.simx_return_ok and detected:
        res, quat = sim.simxGetObjectQuaternion(sim_handle, sensor_handle, -1, sim.simx_opmode_buffer)
        if res == sim.simx_return_ok:
            # https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
            pointing_vec = [0, 0, 0]
            u = quat[0:3]
            s = quat[3]
            # v = [0, 0, 1], implied
            pointing_vec[0] = 2 * u[2] * u[0]
            pointing_vec[1] = 2 * u[2] * u[1]
            pointing_vec[2] = 2 * u[2] * u[2]
            pointing_vec[2] += s * s - u[0] * u[0] - u[1] * u[1] - u[2] * u[2]

            pointing_vec[0] += 2 * s * u[1]
            pointing_vec[1] += 2 * s * -1 * u[0]
            distance = pointing_vec[0] * det_point[0] + pointing_vec[1] * det_point[1] + \
                pointing_vec[2] * det_point[2]
            return abs(distance)
    return MAX_DISTANCE


