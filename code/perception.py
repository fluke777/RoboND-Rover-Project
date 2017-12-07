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

# This function performs transformation. We basically mark 4 points in the source picture and
# tell the function where those pictures should be in the resulting image.
def warp(image):
    source = np.float32([[14, 140], [305, 140], [204, 98], [115, 98]])
    dst_size = 5 
    bottom_offset = 6
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                      [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                      ])


    return perspect_transform(image, source, destination)

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    world_size = Rover.worldmap.shape[0]
    scale = 10

    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    warped = warp(Rover.img)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    img = np.zeros([160,320,3],dtype=np.uint8)

    # I had trouble detecting samples directly in RGB so I turned into HSV where
    # I had more success
    hsv = cv2.cvtColor(warped, cv2.COLOR_RGB2HSV)
    # you can actually define two colors and search in range
    lower_y = np.array([25,41,150])
    upper_y = np.array([50,250,255])
    mask = cv2.inRange(hsv, lower_y, upper_y)
    # pick the passable terrain
    colorsel = color_thresh(warped, rgb_thresh=(160, 160, 160))
    # invert basically. What is not passable is impassable
    rocks = colorsel * -1 +1

    img[:,:,0] = mask
    img[:,:,1] = colorsel * 255
    img[:,:,2] = (rocks) * 255
    # Update the segmented view of the robot (lower left above camera)
    Rover.vision_image = img

    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(colorsel)
    samples_xpix, samples_ypix = rover_coords(mask)
    obstacles_xpix, obstacles_ypix = rover_coords(rocks)

    # 6) Convert rover-centric pixel values to world coordinates
    rover_centric_samples_distances, rover_centric_samples_angles = to_polar_coords(samples_xpix, samples_ypix)
    world_centric_x, world_centric_y = pix_to_world(samples_xpix, samples_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)

    # Here we check if we found some sample 
    if len(rover_centric_samples_angles[rover_centric_samples_angles > 0]) > 0 :
        Rover.seeing_sample = True
        Rover.sample_angle = np.mean(rover_centric_samples_angles * 180/np.pi)        
        # print("===========")
        # print("FOUND STUFF")
        # print("===========")
        #  If we did we will switch to "approaching mode"
        Rover.mode = 'closing_to_sample'

        # mark on the map
        rock_idx = np.argmin(rover_centric_samples_distances)
        try:
            rock_x = world_centric_x[rock_idx]
            rock_y = world_centric_y[rock_idx]
            Rover.worldmap[rock_y, rock_x, 1] = 255
        except:
            print "retry"       
    else:
        Rover.seeing_sample = False
        Rover.sample_angle = None


    rover_xpos = Rover.pos[0]
    rover_ypos = Rover.pos[1]
    rover_yaw = Rover.yaw

    # Get navigable pixel positions in world coords
    navigable_x_world, navigable_y_world = pix_to_world(xpix, ypix, rover_xpos, 
                                           rover_ypos, rover_yaw, 
                                           world_size, scale)


    obstacles_x_world, obstacles_y_world = pix_to_world(obstacles_xpix, obstacles_ypix, rover_xpos, 
                                           rover_ypos, rover_yaw, 
                                           world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: 
    # Generate 200 x 200 pixel worldmap
    worldmap = np.zeros((200, 200))

    # Update the map only if we are in a leveled position so the transformation is valid
    # This helps the accuracy of representation a bit
    if (Rover.pitch >= 359.5 or Rover.pitch <= 0.5) and (Rover.roll >= 359.5 or Rover.roll <= 0.5):
        # add value to obstacles. This is implementing sort of voting.
        # If we conistently find something to be obstacle/passable eventually we will deem it that
        worldmap[obstacles_y_world, obstacles_x_world] -= 1
        worldmap[navigable_y_world, navigable_x_world] += 1


    vfunc = np.vectorize(lambda t: 0 if (t == 0) else 1/t)
    passable_terrain   = vfunc(worldmap) * (worldmap > 0).astype(float) * worldmap
    impassable_terrain = vfunc(worldmap) * (worldmap <= 0).astype(float) * worldmap

    # Update the picture itself 
    Rover.worldmap[:, :, 0] += impassable_terrain
    Rover.worldmap[:, :, 2] += passable_terrain

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(xpix, ypix)
    Rover.nav_dists = rover_centric_pixel_distances
    Rover.nav_angles = rover_centric_angles
    
    return Rover