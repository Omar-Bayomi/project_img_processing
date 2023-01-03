import numpy as np
import math
import time

def stop_at_pos(Rover, mag, direction, max_mag):
            nearest_nav_angle = direction
            if(mag > 1):
                i = get_nearest_index(Rover.nav_angles, direction)
                if len(Rover.nav_angles)>0:
                    nearest_nav_angle = Rover.nav_angles[i]
            if(mag<0.01):
            	direction = 0
            if abs(direction) > 15:
                if Rover.vel > 0.2:
                    Rover.throttle = 0
                    Rover.brake = 0.1
                    if nearest_nav_angle > 0:
                        Rover.steer = np.clip(abs(nearest_nav_angle), -15, 15)
                    else:
                        Rover.steer = -np.clip(abs(nearest_nav_angle), -15, 15)
                else:
                    if nearest_nav_angle > 0:
                        Rover.steer = np.clip(abs(nearest_nav_angle), -15, 15)
                    else:
                        Rover.steer = -np.clip(abs(nearest_nav_angle), -15, 15)
                    Rover.throttle = 0
                    Rover.brake = 0
            elif mag > max_mag:
                Rover.throttle = 0
                Rover.brake = 0
                if Rover.vel > 1.5:
                    Rover.brake = 0.1
                elif Rover.vel < 1:
                    Rover.throttle = 0.2
                if nearest_nav_angle > 0:
                    Rover.steer = np.clip(abs(nearest_nav_angle), -15, 15)
                elif nearest_nav_angle < 0:
                    Rover.steer = -np.clip(abs(nearest_nav_angle), -15, 15)
                else:
                    Rover.steer = 0
            else: stop_rover(Rover)
            



def get_direction_vector(x1, y1, x2, y2):
    if x1 == x2:
    	if y1 == y2:
    	    return 0, 0
    	else:
    	    direction = ((y2-y1)/abs(y2-y1)) * 90
    else: 
        direction = math.atan((y2-y1)/(x2-x1)) *180/np.pi
    mag = math.sqrt((y2-y1)**2 + (x2-x1)**2)
    if x2-x1 < 0:
    	direction = 180 + direction
    	if direction > 180:
    	    direction = direction - 360
    return mag, direction
    
    
    
def normalize_angle(angle):
    normalized = angle
    while(normalized > 180):
        normalized = normalized - 360
    while(normalized < -180):
        normalized = normalized + 360
    return normalized
    
    
        
def get_nearest_index(values_list, value):
    angle_idx = 0
    for i in range(len(values_list)):
        if abs(values_list[i]-value) < abs(values_list[angle_idx]-value):
            angle_idx = i
    return angle_idx
    
    
    
def stop_rover(Rover):
    if Rover.vel > 0:
        Rover.throttle = 0
        Rover.brake = 1
        Rover.steer = 0
    else:
        Rover.brake = 0
        Rover.steer = 0
        


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        n = 0   #this n makes the mean angle weighted by the distance
        weighted_angles = (Rover.nav_dists**n * Rover.nav_angles)
        
        ################################################################################
        if Rover.back_home == True:
            mag, direction = get_direction_vector(Rover.pos[0], Rover.pos[1], Rover.init_x, Rover.init_y)
            print(direction)
            direction = direction - Rover.yaw
            direction = normalize_angle(direction)
            print(direction)
            print(mag)
            weighted_angles.sort()
            if(mag < 0.5):
            	Rover.finished = True   
            elif abs(direction) > 130:
                if Rover.rotate_backward == False:
            	    Rover.backward_yaw = Rover.yaw
            	    Rover.finished = False
            	    if (Rover.backward_yaw<20):
            	    	Rover.last_steer = 1
            	    elif Rover.backward_yaw >340:
            	        Rover.last_steer = -1
            	    else:
            	        Rover.last_steer = -1 * Rover.last_steer
            	    Rover.rotate_backward = True
            elif direction <-20:
            	weighted_angles = weighted_angles[0:int(len(weighted_angles)* 0.75)]
            	Rover.finished = False
            elif direction > 20:
            	weighted_angles = weighted_angles[int(len(weighted_angles)* 0.25):int(len(weighted_angles))-1]
            	Rover.finished = False
            
        ################################################################################

        
        if sum(Rover.nav_dists**n) == 0 or len(weighted_angles) == 0:
        	mean_angle = math.pi / 4
        else:	
        	#mean_angle = sum(weighted_angles) / sum(Rover.nav_dists**n)
        	mean_angle = sum(weighted_angles) / len(weighted_angles)
        angle_idx = 0
        for i in range(len(Rover.nav_angles)):
            if abs(Rover.nav_angles[i]) < abs(Rover.nav_angles[angle_idx]):
                angle_idx = i
        # Check for Rover.mode status
	#############################################################################
        print(Rover.rotate_backward)
        if Rover.finished == True:
            stop_at_pos(Rover, mag, direction, 0.05)
        elif Rover.rotate_backward == True:
            if Rover.vel > 0.2:
            	stop_rover(Rover)
            else:
                Rover.brake = 0
                if (abs(Rover.yaw - Rover.backward_yaw) < 180):
                    Rover.steer = Rover.last_steer * 15
                    Rover.brake = 0
                    print(Rover.last_steer)
                    print(Rover.backward_yaw)
                else:
                    Rover.rotate_backward = False
                    Rover.steer = 0
        #############################################################################
        elif Rover.rock_found == True:
            direction = Rover.rock_angle
            mag = Rover.rock_dist
            if(mag > 20):
                i = get_nearest_index(Rover.nav_angles, direction)
                if len(Rover.nav_angles)>0:
                    direction = Rover.nav_angles[i]
            if abs(direction) > 0.8 and mag <50:
                if Rover.vel > 0.2:
                    Rover.throttle = 0
                    Rover.brake = 0.1
                    if direction > 0:
                        Rover.steer = np.clip(abs(direction)*180/np.pi, -15, 15)
                    else:
                        Rover.steer = -np.clip(abs(direction)*180/np.pi, -15, 15)
                else:
                    if direction > 0:
                        Rover.steer = np.clip(abs(direction)*180/np.pi, -15, 15)
                    else:
                        Rover.steer = -np.clip(abs(direction)*180/np.pi, -15, 15)
                    Rover.throttle = 0
                    Rover.brake = 0
            elif mag > 18:
                Rover.throttle = 0
                Rover.brake = 0
                if Rover.vel > 1:
                    Rover.brake = 0.1
                elif Rover.vel < 1:
                    Rover.throttle = 0.2
                if direction > 0:
                    Rover.steer = np.clip(abs(direction)*180/np.pi, -15, 15)
                elif direction < 0:
                    Rover.steer = -np.clip(abs(direction)*180/np.pi, -15, 15)
                else:
                    Rover.steer = 0
            else:
                stop_rover(Rover)
            
            #####################################################################
        elif len(Rover.nav_dists) == 0 or abs(Rover.nav_angles[angle_idx]) > 0.15:
        	Rover.throttle = 0
        	Rover.brake = 0
        	Rover.steer = 15 #np.clip(mean_angle * 180/np.pi, -15, 15)
        elif Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if Rover.nav_dists[angle_idx] >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(mean_angle * 180/np.pi, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif Rover.nav_dists[angle_idx] < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if Rover.nav_dists[angle_idx] < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if Rover.nav_dists[angle_idx] >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(mean_angle * 180/np.pi, -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    if Rover.picking_up:
    	Rover.rock_found = False
    
    return Rover

