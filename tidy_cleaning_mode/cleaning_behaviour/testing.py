green_boxes = [[2.0, 1.0, -1.0009104013442993, -0.3831886947154999], [4.0, 1.0, 0.8232371211051941, 0.022076798602938652]]#,[1.0,1.0,-1.0,-0.52]
red_boxes = [[1.0, 0.0, -0.9871197938919067, 0.7604538202285767], [3.0, 0.0, 0.8592796921730042, -0.4166344106197357]]#[2.0,0.0,-0.87,0.80],
# [2.0, 1.0, -1.0009104013442993, -0.3831886947154999], [4.0, 1.0, 0.8232371211051941, 0.022076798602938652], 
box_s = 12
target_box =[3.0, 0.0, 0.8592796921730042, -0.4166344106197357]



import global_pathplanning as gp

pathplanning = gp.PathPlanning(green_boxes,red_boxes)
valid_path = pathplanning.PathMakingBackup(0.0,0.0,target_box)
print("the target box is valid? ",valid_path)
path2Start = pathplanning.Nav2Start(0,0,green_boxes,red_boxes)
print("The path to the start of the path is ",path2Start)
pathplanning.Pix2Cart(pathplanning.push)


# import cv2
# import numpy as np

# # Read the image
# image = cv2.imread('/workspaces/AMR-Assessment/World_map.pgm',cv2.COLOR_BGR2GRAY)
# height, width = image.shape[:2]

# #resolution makes 1 pixel = 0.5cm
# x_factor = 600/width
# y_factor = 600/height
        
# image = cv2.resize(image, None, fx=x_factor, fy=y_factor, interpolation=cv2.INTER_NEAREST)
# height, width = image.shape[:2]
# # Define the coordinates of the region you want to erode (x, y, width, height)
# x = 40  # Starting x-coordinate of the region
# y = 40  # Starting y-coordinate of the region
# width = 520  # Width of the region
# height = 520  # Height of the region
# a, binary_map = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
# # Create a binary mask specifying the region
# mask = np.zeros_like(binary_map[:, :])  # Create a mask with the same dimensions as the image
# mask[y:y+height, x:x+width] = 255  # Set the region to 255 (white)

# # Apply erosion to the mask
# kernel = np.ones((30, 30), np.uint8)
# eroded_mask = cv2.erode(mask, kernel, iterations=1)
# wall_mask = cv2.bitwise_and(cv2.bitwise_not(binary_map), cv2.bitwise_not(binary_map), mask=eroded_mask)
# oppi_wal  = cv2.bitwise_not(wall_mask)

# kernel = np.ones((60, 60), np.uint8)
# expanded_black_pixels_mask = cv2.erode(oppi_wal, kernel, iterations=1)
# new_mask = cv2.bitwise_and(expanded_black_pixels_mask, eroded_mask)
# # Apply the expanded black pixels mask to the original image
# #result = cv2.bitwise_and(binary_map, binary_map, mask=expanded_black_pixels_mask)


# cv2.circle(new_mask, (300,55), 5, (255, 255,255 ), -1)
# cv2.circle(image, (300,10), 5, (255, 255,255 ), -1)
# # Display the result
# cv2.imshow('Original Image', image)
# cv2.imshow('wall mask', eroded_mask)
# cv2.imshow('obj mask', expanded_black_pixels_mask)
# cv2.imshow('combined', new_mask)
# #cv2.imshow('Expanded Black Pixels', result)
# cv2.waitKey(0)
# cv2.destroyAllWindows()