import numpy as np
import cv2, copy, math

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    color_select2 = np.ones_like(img[:,:,0])
    
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    color_select2[above_thresh] = 0
    # Return the binary image
    return color_select2, color_select
def color_thresh2(img, rgb_thresh=(0, 0, 0), high_thresh=(210, 205, 90)):
    ###### TODO:
    # Create an empty array the same size in x and y as the image 
    # but just a single channel
    color_select = np.zeros_like(img[:,:,0])
    rock_thresh = (img[:, :, 0] >= rgb_thresh[0]) & (img[:, :, 0] < high_thresh[0])  & (img[:,:,1] >= rgb_thresh[1]) & (img[:,:,1] < high_thresh[1]) & (img[:,:,2] >= rgb_thresh[2]) & (img[:,:,2] < high_thresh[2])
    
    color_select[rock_thresh] = 1
                
                    
                
    # Apply the thresholds for RGB and assign 1's 
    # where threshold was exceeded
    # Return the single-channel binary image
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


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    
    img = Rover.img  
    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    warped = perspect_transform(img, source, destination)
    
    #plt.subplot(224)
    #plt.imshow(img)
    obs_threshed, path_threshed = color_thresh(warped, (160, 160, 160))
    #obs_threshed = color_thresh_rest(warped, (160, 160, 160))
    rock_threshed = color_thresh2(warped, (125, 105, -1))
    Rover.vision_image = copy.deepcopy(img)
    
    #obs_threshed = perspect_transform(obs_threshed, source, destination)
    #path_threshed = perspect_transform(path_threshed, source, destination)
    #rock_threshed = perspect_transform(rock_threshed, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    #obs_threshed -= rock_threshed
    #t2 = np.copy(obs_threshed)
    #plt.subplot(221)
    #plt.imshow(path_threshed, cmap='gray')
    #plt.subplot(222)
    #plt.imshow(obs_threshed, cmap='gray')
    #plt.subplot(223)
    #plt.imshow(rock_threshed, cmap='gray')
    
    for y in range(len(obs_threshed)):
        for x in range(len(obs_threshed[y])):
            if rock_threshed[y][x] == obs_threshed[y][x]:
                obs_threshed[y][x] = 0
    # 4) Convert thresholded image pixel values to rover-centric coords
    #obs_threshed = perspect_transform(obs_threshed, source, destination)
    #path_threshed = perspect_transform(path_threshed, source, destination)
    #rock_threshed = perspect_transform(rock_threshed, source, destination)
    
    xpix, ypix = rover_coords(path_threshed)
    rxpix, rypix = rover_coords(rock_threshed)
    oxpix, oypix = rover_coords(obs_threshed)
    
    dist, angles = to_polar_coords(xpix, ypix)
    
    rdist, rangles = to_polar_coords(rxpix, rypix)
    
    odist, oangles = to_polar_coords(oxpix, oypix)
    
    Rover.obs_angles = oangles
    Rover.obs_dists = odist

    Rover.nav_angles = angles
    Rover.nav_dists = dist
    #avg = np.average(angles, weights=dist)




    

    #dist, angles = to_polar_coords(xpix, ypix)
    # 5) Convert rover-centric pixel values to world coords
    
    x, y = Rover.pos
    yaw = Rover.yaw
    
    x_pix_world, y_pix_world = pix_to_world(xpix, ypix, x, y, yaw, 200, 10)
    rx_pix_world, ry_pix_world = pix_to_world(rxpix, rypix, x, y, yaw, 200, 10)
    ox_pix_world, oy_pix_world = pix_to_world(oxpix, oypix, x, y, yaw, 200, 10)
    # 6) Update worldmap (to be displayed on right side of screen)
        # Example: data.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          data.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          data.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    
    #print(x, y)
    #print(ox_pix_world)
    #print(x_pix_world)
    #print(rx_pix_world)
    Rover.rock_angles = []
    Rover.rock_dists = []
    
    for i in range(len(ox_pix_world)):
        if odist[i] < 150:
            Rover.worldmap[oy_pix_world[i], ox_pix_world[i], 0] += 1
            
    for i in range(len(rx_pix_world)):
        if rdist[i] < 70:
            Rover.rock_angles.append(rangles[i])
            Rover.rock_dists.append(rdist[i])
            Rover.worldmap[ry_pix_world[i], rx_pix_world[i], 1] += 10
            #Rover.samples_located += 1
            '''
            print("le wild ", Rover.samples_pos)
            if Rover.samples_pos == None:
            	print("pre ",Rover.samples_pos)
            	print("vals ",(rx_pix_world[i], ry_pix_world[i],))
            	Rover.samples_pos = [(rx_pix_world[i], ry_pix_world[i],)]
            	print("post ",Rover.samples_pos)

            else:
            	if (rx_pix_world[i], ry_pix_world[i]) not in Rover.samples_pos:
            		print("pre ",Rover.samples_pos)
            		print("vals ",(rx_pix_world[i], ry_pix_world[i],))
            		Rover.samples_pos.append((rx_pix_world[i], ry_pix_world[i],))
            		print("post ",Rover.samples_pos)
            '''		
            Rover.worldmap[ry_pix_world[i], rx_pix_world[i], 0] = 0
            Rover.worldmap[ry_pix_world[i], rx_pix_world[i], 2] = 0
            
    for i in range(len(x_pix_world)):
        if dist[i] < 70:
            Rover.worldmap[y_pix_world[i], x_pix_world[i], 2] += 5
            Rover.worldmap[y_pix_world[i], x_pix_world[i], 0] = 0

    # 7) Make a mosaic image, below is some example code
        # First create a blank image (can be whatever shape you like)
    #output_image = np.zeros((img.shape[0] + data.worldmap.shape[0], img.shape[1]*2, 3))
        # Next you can populate regions of the image with various output
        # Here I'm putting the original image in the upper left hand corner
    #output_image[0:img.shape[0], 0:img.shape[1]] = img

        # Let's create more images to add to the mosaic, first a warped image
    #warped = perspect_transform(img, source, destination)
        # Add the warped image in the upper right hand corner
    #output_image[0:img.shape[0], img.shape[1]:] = warped

        # Overlay worldmap with ground truth map
    #map_add = cv2.addWeighted(data.worldmap, 1, data.ground_truth, 0.5, 0)
        # Flip map overlay so y-axis points upward and add to output_image 
    #output_image[img.shape[0]:, 0:data.worldmap.shape[1]] = np.flipud(map_add)


        # Then putting some text over the image
    #cv2.putText(output_image,"Populate this image with your analyses to make a video!", (20, 20), 
    #            cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
    #if data.count < len(data.images) - 1:
    #    data.count += 1 # Keep track of the index in the Databucket()
    
    #return output_image
    Rover.vision_image = path_threshed
    return Rover
