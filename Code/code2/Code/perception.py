import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

# def color_thresh(img, rgb_thresh=(160, 160, 160)):
#     color_select = np.zeros_like(img[:,:,0])
#     above_thresh = (img[:,:,0] > rgb_thresh[0]) \
#                 & (img[:,:,1] > rgb_thresh[1]) \
#                 & (img[:,:,2] > rgb_thresh[2])
#     color_select[above_thresh] = 1
#     return color_select

def rock_thresh(img, rgb_thresh=(100, 100, 30)):
    color_select = np.zeros_like(img[:,:,0])
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    color_select[above_thresh] = 1
    return color_select


# Apply the above functions in succession and update the Rover state accordingly

    #two pipeline one to draw on map and the other to be sent to decision
    
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 7
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable_terrain = color_thresh(warped)
    obstacles = np.abs(navigable_terrain - 1) * perspect_transform((warped * 0 + 1)[:,:,0], source, destination)
    rock = rock_thresh(warped)
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacles
    Rover.vision_image[:,:,1] = rock * 255
    Rover.vision_image[:,:,2] = navigable_terrain * 255

    # 5) Convert map image pixel values to rover-centric coords
    xpix_obs, ypix_obs = rover_coords(obstacles)
    xpix_rock, ypix_rock = rover_coords(rock)
    xpix_ter, ypix_ter = rover_coords(navigable_terrain)
    xpos, ypos = Rover.pos
    
    #############################################################################################
    if len(xpix_ter) != 0 :
        Rover.max = max(ypix_ter)
        Rover.min = min(ypix_ter)
        coord_list = list(zip(xpix_ter , ypix_ter))
        for tuple_i in coord_list:
            xpix = tuple_i[0]
            ypix = tuple_i[1]
            if xpix > 80:
                coord_list.remove((xpix, ypix))
        
        xpix_ter_clipped , ypix_ter_clipped = zip(*coord_list)
        Rover.terrain_width = max(ypix_ter_clipped) - min(ypix_ter_clipped)
        #ypix_ter.sort()
        #Rover.min = ypix_ter[0]
    count = 0
    Rover.notch = False
    if len(xpix_obs) != 0 :
        print("max", max(xpix_obs), "========================================")
        print("min", min(xpix_obs), "========================================")
        coord_list_obs = list(zip(xpix_obs, ypix_obs))
        for x_pix, y_pix in coord_list_obs:
            if x_pix < 16 and y_pix < 0:
                count = count + 1
        if count >= 50:
            Rover.notch = True
    ################################################################################################
    
    # 6) Convert rover-centric pixel values to world coordinates
    obstacle_x_world, obstacle_y_world = pix_to_world(xpix_obs, ypix_obs, xpos, ypos, Rover.yaw, 200, dst_size*2)
    rock_x_world, rock_y_world = pix_to_world(xpix_rock, ypix_rock, xpos, ypos, Rover.yaw, 200, dst_size*2)
    terrain_x_world, terrain_y_world = pix_to_world(xpix_ter, ypix_ter, xpos, ypos, Rover.yaw, 200, dst_size*2)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1

    #Rover.worldmap[terrain_y_world, terrain_x_world, 0] = 0
    #Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    #Rover.worldmap[obstacle_y_world, obstacle_x_world, 2] = 0

    # 8) Convert rover-centric pixel positions to polar coordinates
    dists, angles = to_polar_coords(xpix_ter, ypix_ter)
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists = dists
    Rover.nav_angles = angles
    
    ##############################################################################
    if len(rock_y_world) >= 1 and len(rock_x_world) >= 1:
        Rover.rock_found = True
        dist, angle = to_polar_coords(xpix_rock, ypix_rock)
        Rover.rock_dist = dist[0]
        Rover.rock_angle = angle[0]
    ##############################################################################
    
    #anothor pipeline for map
    map_dst_size = 11
    map_destination = np.float32([[Rover.img.shape[1]/2 - map_dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + map_dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + map_dst_size, Rover.img.shape[0] - 2*map_dst_size - bottom_offset], 
                      [Rover.img.shape[1]/2 - map_dst_size, Rover.img.shape[0] - 2*map_dst_size - bottom_offset],
                      ])
    
    map_warped = perspect_transform(Rover.img, source, map_destination)
    
    map_warped = cv2.GaussianBlur(map_warped,(101,101),0)
    
    map_navigable_terrain = color_thresh(map_warped)
    
    kernel = np.ones((5, 5), np.uint8)
    map_navigable_terrain = cv2.erode(map_navigable_terrain, kernel, iterations=1)

    map_xpix_ter, map_ypix_ter = rover_coords(map_navigable_terrain)
    
    map_terrain_x_world, map_terrain_y_world = pix_to_world(map_xpix_ter, map_ypix_ter, xpos, ypos, Rover.yaw, 200, map_dst_size*2)
    
    if ((Rover.pitch < 0.8 and Rover.pitch >= 0) or (Rover.pitch <= 360 and Rover.pitch > 359.6)) and ((Rover.roll < 1 and Rover.roll >= 0) or (Rover.roll <= 360 and Rover.roll > 359)):
    	Rover.worldmap[map_terrain_y_world, map_terrain_x_world, 2] += 1
    	Rover.worldmap[map_terrain_y_world, map_terrain_x_world, 0] = 0
    	
    	
    	
    ##########################################################################################
    while (Rover.init_x == None or Rover.init_y == None or Rover.init_yaw == None):
    	Rover.init_x, Rover.init_y = Rover.pos
    	Rover.init_yaw = Rover.yaw
    if (Rover.samples_collected >= 5 and Rover.percent_mapped > 95):
        Rover.rock_found = False
        Rover.back_home = True
     	
    #####################################################################################
    return Rover
