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
            particle_after_motion_update = Particle(x + dx_new, y + dy_new, h + dh_new)

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
    
    weight_of_particles = []
    probability_of_particles = []

    normalizing_factor = 0.0
        
    #calculate weights    
    for particle in particles:
            #if not in grid, weight = 0.0
            if not grid.is_in(particle.x, particle.y) or not grid.is_free(particle.x, particle.y):
                    weight_of_particles.append(0.0)
            else:
                    probability = calculate_probability(particle, measured_marker_list, grid)
                    weight_of_particles.append(probability)
                    normalizing_factor += probability
    
    #spurious detection rate and detection failure rate
    if (len(particles) > len(measured_marker_list)):
            #the number of dropped markers = number of markers seen by the particle - number of markers seen by the robot
            dropped_markers =  len(particles) - len(measured_marker_list)
            scale = dropped_markers * setting.DETECTION_FAILURE_RATE
            weight_of_particles = [weight * scale for weight in weight_of_particles]
    else:
            spurious_markers =  len(measured_marker_list) - len(particles)
            scale = spurious_markers * setting.SPURIOUS_DETECTION_RATE
            weight_of_particles = [weight * scale for weight in weight_of_particles]
            
        
    #normalize
    if normalizing_factor == 0.0: 
            # equal probabilities
            probability_of_particles = [1 / len(weight_of_particles) for weight in weight_of_particles]
    else: 
            probability_of_particles = [weight / normalizing_factor for weight in weight_of_particles]

    
    #account for addition of random samples 
    random_sample_size = int(len(particles) / 8)
    
    #resample
    measured_particles = np.random.choice(particles, size=len(particles) - random_sample_size, replace=True, p=probability_of_particles)
    #convert from numpy array to list
    measured_particles = measured_particles.tolist()
    
    #add small number of random particles 
    random_particles = Particle.create_random(count=random_sample_size, grid=grid)
    measured_particles = measured_particles + random_particles

    return measured_particles

# not given method
def calculate_probability(particle, measured_marker_list, grid):
    probability = 0.0 
    return probability
    





