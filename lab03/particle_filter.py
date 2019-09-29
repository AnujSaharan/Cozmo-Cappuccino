from grid import *
from particle import Particle
from utils import *
import setting
import numpy as np
import random
np.random.seed(setting.RANDOM_SEED)
random.seed(setting.RANDOM_SEED)


def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []

    dx, dy, dh = odom #given - delta x, y, angle
    for particle in particles: 
            x, y, h = particle.xyh

            #rotate odom frame to match particle frame
            dx_rotated, dy_rotated = rotate_point(dx, dy, h)
            odom_new = (dx_rotated, dy_rotated, dh)

            #add gaussian noise to odom reading
            dx_new, dy_new, dh_new = add_odometry_noise(odom_new, setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)
            
            #increment particle x, y, h by odom deltas
            particle_after_motion_update = Particle(x+dx_new, y+dy_new, h+dh_new)

            motion_particles.append(particle_after_motion_update)

    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    measured_particles = []
    return measured_particles


