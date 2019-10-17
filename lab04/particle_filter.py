from grid import *
from particle import Particle
from utils import *
<<<<<<< HEAD
import setting
import numpy as np
np.random.seed(setting.RANDOM_SEED)
=======
import setting 
import numpy as np
np.random.seed(RANDOM_SEED)
>>>>>>> cac2f6b70fe2a01d22d2d072cbe666c73ebd93ea
from itertools import product


def motion_update(particles, odom):
    """ Particle filter motion update
<<<<<<< HEAD
=======

>>>>>>> cac2f6b70fe2a01d22d2d072cbe666c73ebd93ea
        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*
<<<<<<< HEAD
=======

>>>>>>> cac2f6b70fe2a01d22d2d072cbe666c73ebd93ea
        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []

<<<<<<< HEAD
    for particle in particles:
        x, y, h = particle.xyh
        dx, dy, dh = odom
        c, d = rotate_point(dx, dy, h)
        nx, ny, nh = add_odometry_noise((x+c, y+d, h+dh), heading_sigma=setting.ODOM_HEAD_SIGMA, trans_sigma=setting.ODOM_TRANS_SIGMA)
        newParticle = Particle(nx, ny, nh%360)
        motion_particles.append(newParticle)
=======
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
            particle_after_updating = Particle(x + dx, y + dy, h + h_new)
            motion_particles.append(particle_after_updating)
>>>>>>> cac2f6b70fe2a01d22d2d072cbe666c73ebd93ea

    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update
<<<<<<< HEAD
        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)
=======

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

>>>>>>> cac2f6b70fe2a01d22d2d072cbe666c73ebd93ea
        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree
<<<<<<< HEAD
                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one
        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles
        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    num_random_sample = 25
    measured_particles = []
    weight = []

    if len(measured_marker_list) > 0:
        for particle in particles:
            x, y = particle.xy
            if grid.is_in(x, y) and grid.is_free(x, y):
                markers_visible_to_particle = particle.read_markers(grid)
                markers_visible_to_robot = measured_marker_list.copy()

                marker_pairs = []
                while len(markers_visible_to_particle) > 0 and len(markers_visible_to_robot) > 0:
                    all_pairs = product(markers_visible_to_particle, markers_visible_to_robot)
                    pm, rm = min(all_pairs, key=lambda p: grid_distance(p[0][0], p[0][1], p[1][0], p[1][1]))
                    marker_pairs.append((pm, rm))
                    markers_visible_to_particle.remove(pm)
                    markers_visible_to_robot.remove(rm)

                prob = 1.
                for pm, rm in marker_pairs:
                    d = grid_distance(pm[0], pm[1], rm[0], rm[1])
                    h = diff_heading_deg(pm[2], rm[2])

                    exp1 = (d**2)/(2*setting.MARKER_TRANS_SIGMA**2)
                    exp2 = (h**2)/(2*setting.MARKER_ROT_SIGMA**2)

                    likelihood = math.exp(-(exp1+exp2))
                    # The line is the key to this greedy algorithm
                    # prob *= likelihood
                    # print(setting.DETECTION_FAILURE_RATE)
                    prob *= max(likelihood, setting.DETECTION_FAILURE_RATE*setting.SPURIOUS_DETECTION_RATE)

                # In this case, likelihood is automatically 0, and max(0, DETECTION_FAILURE_RATE) = DETECTION_FAILURE_RATE
                prob *= (setting.DETECTION_FAILURE_RATE**len(markers_visible_to_particle))
                # Probability for the extra robot observation to all be spurious
                prob *= (setting.SPURIOUS_DETECTION_RATE**len(markers_visible_to_robot))
                weight.append(prob)

            else:
                weight.append(0.)
    else:
        weight = [1.]*len(particles)

    norm = float(sum(weight))

    if norm != 0:
        weight = [i/norm for i in weight]
        measured_particles = Particle.create_random(num_random_sample, grid)
        measured_particles += np.random.choice(particles, setting.PARTICLE_COUNT-num_random_sample, p=weight).tolist()
    else:
        measured_particles = Particle.create_random(setting.PARTICLE_COUNT, grid)

    return measured_particles
=======

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
>>>>>>> cac2f6b70fe2a01d22d2d072cbe666c73ebd93ea
