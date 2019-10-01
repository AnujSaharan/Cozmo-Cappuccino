# CS 3630 - Lab 3
# Anuj Saharan
# Riya Agrawal

from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np

np.random.seed(setting.RANDOM_SEED)

def motion_update(particles, odom):
    motion_particles = []
    
    dx, dy, dh = odom
    
    if dx == 0 and dy == 0 and dh == 0: # Particles shouldn't move if the robot doesn't move
        return particles
    
    for particle in particles: 
            x, y, h = particle.xyh

            # Add Gaussian noise to odometer data
            x_new, y_new, h_new = add_odometry_noise(odom, setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)
            
            # Rotate to the current particle head angle
            dx, dy = rotate_point(x_new, y_new, h)
            
            # New Particle
            updated_particle = Particle(x + dx, y + dy, h + h_new)
            motion_particles.append(updated_particle)
    return motion_particles

def measurement_update(particles, measured_marker_list, grid):
    measured_particles = []
    
    if len(measured_marker_list) <= 0: # If there are no markers seen, do not update measurement
        return particles

    weights = []

    for particle in particles:        
        # If the particle is not on the map, give it a weight of zero
        if not grid.is_in(particle.x, particle.y) or not grid.is_free(particle.x, particle.y):
            weights.append(0)
            continue

        probability = 1.0
        
        # If there are no markers in the secondary list, but they are in actual measurement, do not use this particle
        secondary_marker_list = particle.read_markers(grid)

        if len(secondary_marker_list) <= 0:
            weights.append(0)
            continue

        # Pair observarble markets by distance
        for measured_marker in measured_marker_list:
            highest_probability = -0.1
            max_probability_pair = None

            # Current Particle observes fewer markers than the robot
            if len(secondary_marker_list) <= 0:
                break

            for observed_marker in secondary_marker_list:
                
                distance_between_markers = math.sqrt(measured_marker[0]**2 + measured_marker[1]**2) - math.sqrt(observed_marker[0]**2 + observed_marker[1]**2)
                
                angle_between_markers = diff_heading_deg(measured_marker[2], observed_marker[2])
                
                current_probability = np.exp(-(distance_between_markers**2)/(2*MARKER_TRANS_SIGMA**2)
                                   - (angle_between_markers**2)/(2*MARKER_ROT_SIGMA**2))
                
                if current_probability > highest_probability:
                    highest_probability = current_probability
                    max_probability_pair = observed_marker

            if max_probability_pair != None:
                secondary_marker_list.remove(max_probability_pair)

            probability *= highest_probability
        weights.append(probability)

    # Normalize all current weights
    weights = np.divide(weights, np.sum(weights))

    # Resample all particles, 3% random noise
    measured_particles = np.random.choice(
                            particles,
                            size = PARTICLE_COUNT - int(PARTICLE_COUNT*0.03),
                            replace = True,
                            p = weights)

    # Generate random noise to help with the kidnapped robot problem, 3% of actual number of particles
    measured_particles = np.ndarray.tolist(measured_particles) \
                            + Particle.create_random(int(PARTICLE_COUNT*0.03), grid)

    return measured_particles