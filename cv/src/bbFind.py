#!/usr/bin/env python
import time
import 	glob
import 	numpy as np
import 	cv2
import 	matplotlib.pyplot as plt
import 	matplotlib.image as mpimg
import os
# from	moviepy.editor import VideoFileClip

# Assumes that coordinates start from 0,0 at top left of image
bboxes = [ {'bl': (0, 720), 'tl': (0, 385), 'tr': (370, 380), 'br': (400, 720)},
			{'bl': (370, 290), 'tl': (370, 90), 'tr': (1060, 115), 'br': (1070, 410)} ]

# Define perspective transform functions
def warp(undistorted_image):
	# Define calibration box in source (original) and
	# destination (destination or warped) coordinates
	image_size = undistorted_image.shape[1::-1]
	# Four source coordinates
	src = np.float32([[830,250],
					 [1010,530],
					 [180,530],
					 [360,250]])
	# Four desired coordinates
	dst = np.float32([[1000,155],
					 [1000,680],
					 [400,680],
					 [400,155]])
	# Compute the perspective transform M
	M = cv2.getPerspectiveTransform(src, dst)

	# Create the warped image - uses linear interpolation
	warped = cv2.warpPerspective(undistorted_image, M, image_size,flags = cv2.INTER_LINEAR)

	return warped

def unwarp(warped_image):
	# Define calibration box in source (original) and
	# destination (destination or warped) coordinates
	image_size = warped_image.shape[1::-1]
	# Four source coordinates
	src = np.float32([[830,250],
					 [1010,530],
					 [180,530],
					 [360,250]])
	# Four desired coordinates
	dst = np.float32([[1000,155],
					 [1000,680],
					 [400,680],
					 [400,155]])

	# Compute the inverse perspective transform
	Minv = cv2.getPerspectiveTransform(dst, src)

	# Unwarp the warped image - uses linear interpolation
	unwarped = cv2.warpPerspective(warped_image, Minv, image_size, flags = cv2.INTER_LINEAR)

	return unwarped

# Mask out the pylons detected in bboxes:
def apply_masking(bboxes, binary_image_to_mask):
	for i in range(len(bboxes)):
		start_x = bboxes[i].get('bl')[0]
		fin_x = bboxes[i].get('br')[0]
		start_y = bboxes[i].get('tl')[1]
		fin_y = bboxes[i].get('bl')[1]

		width = fin_x - start_x
		height = fin_y - start_y
		for x in range(width):
			for y in range(height):
				binary_image_to_mask[start_y + y - 1][start_x + x - 1] = 0

# Take two different types of thresholding and combine resulting binaries
def process_image(image, bboxes):
	height, width, channels = image.shape

	undistorted_image = image # TODO: Remove camera distortion (see Medium article)

	# Apply Gaussian to undistorted image to help smooth out grass texture
	kernel_size = 19
	blurred_input = cv2.GaussianBlur(undistorted_image, (kernel_size, kernel_size), 0)

	# Generate Binary 1 by thresholding Saturation channel
	hls_image = cv2.cvtColor(blurred_input,cv2.COLOR_RGB2HLS)
	# plt.imshow(hls_image)
	# plt.show()
	s_channel_image = hls_image[:,:,2]
	thresh = (81,255) # Heuristic
	s_thresholded_image = np.zeros_like(s_channel_image)
	s_thresholded_image[(s_channel_image>thresh[0])&(s_channel_image < thresh[1])] = 1
	# Invert the b/w pixels
	s_thresholded_image = cv2.bitwise_not(s_thresholded_image)
	# plt.imshow(s_thresholded_image, cmap="gray")
	# plt.show()

	# Generate Binary 2 by applying Sobel thresholding
	gray = cv2.cvtColor(blurred_input,cv2.COLOR_RGB2GRAY)
	# Gradient X
	sobelx = cv2.Sobel(gray,cv2.CV_64F,1,0)
	# Gradient Y
	sobely = cv2.Sobel(gray,cv2.CV_64F,0,1)
	abs_sobelxy = np.sqrt(sobelx**2 + sobely**2)
	# sobelxy_scaled = np.uint8(255 * abs_sobelxy / np.max(abs_sobelxy))
	sobelxy_scaled = abs_sobelxy
	thresh_min = 30
	thresh_max = 80
	sobel_thresholded_image = np.zeros_like(sobelxy_scaled)
	sobel_thresholded_image[(sobelxy_scaled >= thresh_min) & (sobelxy_scaled <= thresh_max)] = 1



	# Combine two thresholds
	combined_binary = np.zeros_like(sobel_thresholded_image)
	combined_binary[(s_thresholded_image == 1)|(sobel_thresholded_image == 1)] = 1
	# Apply a median blur to the binary to de-speckle:
	combined_binary = cv2.medianBlur(combined_binary.astype(np.uint8),7)
	# plt.imshow(combined_binary, cmap="gray")
	# plt.show()
	# If we have the bboxes from the object detection, we could apply masking here
	masked = combined_binary

	# Change camera pespective into bird's eye view
	warped_image = warp(masked)
	nongraywarp = warp(image)

	###HOUGH LINE TFM
	# # lines = cv2.HoughLines(warped_image.astype(np.uint8),1,np.pi/180,200)
	# minLineLength = 30
	# maxLineGap = 10
	# lines = cv2.HoughLinesP(warped_image.astype(np.uint8),1,np.pi/180,15,minLineLength,maxLineGap)
	# for x in range(0, len(lines)):
	#     for x1,y1,x2,y2 in lines[x]:
	#         cv2.line(nongraywarp,(x1,y1),(x2,y2),(0,0,255),4) #BGR


	# Creating a three channel  image from our 1 channel binary(B&W) Image
	three_channel_thresholded_image = np.dstack((warped_image,warped_image,warped_image))*255
	return three_channel_thresholded_image



## LINE DRAWING-----------
def draw_sliding_window_right(image):
	# Rotate img
	image = np.rot90(image,k=3,axes=(0,1))
	# plt.imshow(image)
	crop_img = image[1100:1280,0:720]
	histogram = np.sum(crop_img[:,:,0],axis = 0)

	#find peaks that are greater than the average and some standard deviation
	mean = np.mean(histogram, axis = 0)
	std = np.std(histogram, axis = 0)

	#min peak cutoff
	cutoff = mean+std*3

	# plt.imshow(crop_img)
	# plt.show()
	plt.plot(histogram)
	plt.plot([0, len(histogram)], [mean+std*2, mean+std*2], color='k', linestyle='-', linewidth=2)
	plt.show()

	binsize = 10
	maxes = list()
	#find peak in each bin
	for i in range(0,binsize):
		leftbound = int(i*720/binsize)
		rightbound = int((i+1)*720/binsize)
		midpoint = int((leftbound+rightbound)/2)
		binmax = np.amax(histogram[:][leftbound:rightbound],axis=0)
		if (binmax>cutoff):
			maxes.append([midpoint,binmax])

	# Hyperparameters
	# choose the number of sliding windows
	nwindows = 20
	# Set the width of the windows +/- margin
	margin = 100
	# Set the minimum number of the pixels to recenter the windows
	minpix = 50
	# Set height of the windows - based on nwindows above and image shape
	window_height = image.shape[0] // nwindows

	xlist=np.empty(0)
	ylist=np.empty(0)

	# Identify the x and y positions of all nonzero (i.e. activated) pixels in the image
	nonzero = image.nonzero()
	nonzeroy = np.array(nonzero[0])
	nonzerox = np.array(nonzero[1])
	# Current positions to be updated later for each window in nwindows
	for i in range(0,len(maxes)):
		leftx_current = maxes[i][0]

		# Create empty lists to receive left and right lane pixel indices
		left_lane_inds = []

		for window in range(nwindows):
			#Identify window boundaries in x and y
			win_y_low = image.shape[0] - (window+1)*window_height
			win_y_high = image.shape[0] - window*window_height
			win_xleft_low = leftx_current - margin
			win_xleft_high = leftx_current + margin

			# Draw the windows on the visualization image
			cv2.rectangle(image,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2)

			# Identify the nonzero pixels in x and y within the window
			good_left_inds = \
						((nonzeroy >= win_y_low)\
						& (nonzeroy < win_y_high) \
						& (nonzerox >= win_xleft_low) \
						&  (nonzerox < win_xleft_high)).nonzero()[0]
			# Append these indices to the lists
			left_lane_inds.append(good_left_inds)
			#if more than minpix pixels were found , recenter next window on their mean position
			if len(good_left_inds) > minpix:
				leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
		# Concatenate the arrays of indices (previously was a list of lists of pixels)
		left_lane_inds = np.concatenate(left_lane_inds)

		# Extract left and right line pixel positions
		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds]

		# Generate x and y values for plotting
		ploty = np.linspace(0, image.shape[0]-1, image.shape[0])
		try:
			# Fit a second order polynomial to each set of lane points
			left_fit = np.polyfit(lefty, leftx, 2)
			left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
		except TypeError:
			# Avoids an error if `left` and `right_fit` are still none or incorrect
			left_fitx = 1*ploty**2 + 1*ploty

		ylist = np.concatenate((ylist,ploty))
		xlist = np.concatenate((xlist,left_fitx))

	points = np.vstack((xlist,ylist))
	#translate points
	points = np.array([np.add(points[0], -720/2),np.add(points[1],-1280/2)])
	#unrotate points
	theta = np.radians(270)
	R = np.array(((np.cos(theta), np.sin(theta)), (np.sin(theta),np.cos(theta))))
	points = np.matmul(R,points)
	#untranslate points
	points = np.array([np.flip(np.add(points[0], 1280/2)),np.add(points[1],720/2)])

	#plt.clf()
	#plt.imshow(image)
	#plt.scatter(left_fitx,ploty, 0.5,color='yellow')
	#plt.show()

	return points

# Polyfits curve to input thresholded binary image
def draw_sliding_window_left(image):

	# Rotate image CCW by 90
	image = np.rot90(image, k=1, axes=(0,1))

	# Find peaks that are greater than the average and some standard deviation
	# Sum up the number of white pixels present along rows
	histogram = np.sum(image[:,:,0],axis=0)
	mean = np.mean(histogram, axis=0)
	std = np.std(histogram, axis=0)

	# Min peak cutoff
	cutoff = mean+std*3

	# plt.plot([0, len(histogram)], histogram)
	# plt.plot([0, len(histogram)], [cutoff, cutoff], color='k', linestyle='-', linewidth=2)
	# # plt.plot([0, len(histogram)], [cutoff, cutoff], color='k', linestyle='-', linewidth=2)
	# plt.show()

	# Partition histogram into bins and find peak in each bin
	binsize = 10
	# List to store midpoint row number of bin with a peak whos value exceeds cutoff
	maxes = list()
	for i in range(0,binsize):
		leftbound = int(i*720/binsize)
		rightbound = int((i+1)*720/binsize)
		midpoint = int((leftbound+rightbound)/2)
		binmax = np.amax(histogram[:][leftbound:rightbound],axis=0)
		if (binmax>cutoff):
			maxes.append([midpoint,binmax])

	# Hyperparameters
	# Choose the number of sliding windows
	nwindows = 20
	# Set the width of the windows +/- margin
	margin = 50
	# Set the minimum number of the pixels to recenter the windows
	minpix = 50
	# Set height of the windows, based on nwindows above and image shape
	window_height = image.shape[0] // nwindows

	xlist=np.empty(0)
	ylist=np.empty(0)

	# Identify the x and y positions of all nonzero (i.e. activated) pixels in the image
	nonzero = image.nonzero()
	nonzeroy = np.array(nonzero[0])
	nonzerox = np.array(nonzero[1])

	# Current positions to be updated later for each window in nwindows
	for i in range(0,len(maxes)):
		leftx_current = maxes[i][0] # Get current midpoint (from 0 to imageshape[0]-1)

		# Create empty lists to receive left and right lane pixel indices
		left_lane_inds = []

		for window in range(nwindows):
			#Identify window boundaries in x and y
			win_y_low = image.shape[0] - (window+1)*window_height
			win_y_high = image.shape[0] - window*window_height
			win_xleft_low = leftx_current - margin
			win_xleft_high = leftx_current + margin

			# Draw the windows on the visualization image
			# cv2.rectangle(image,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2)

			# Identify the nonzero pixels in x and y within the current window
			good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
			# print("good left indices are: ", ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)))

			# Append these indices to the lists
			left_lane_inds.append(good_left_inds)
			# If recenter threshold exceeded, shift next window on the mean position of all the indices
			if len(good_left_inds) > minpix:
				# print("good left inds exceeded min pix")
				leftx_current = np.int(np.mean(nonzerox[good_left_inds]))

		# Concatenate the arrays of indices (previously was a list of lists of pixels)
		left_lane_inds = np.concatenate(left_lane_inds)

		# Extract left and right line pixel positions
		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds]

		if(len(leftx) <= 50000):
			continue

		# Generate x and y values for plotting
		ploty = np.linspace(0, image.shape[0]-1, image.shape[0])
		try:
			# Fit a second order polynomial to each set of lane points
			left_fit = np.polyfit(lefty, leftx, 2)
			left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
		except TypeError:
			# Avoids an error if `left` and `right_fit` are still none or incorrect
			left_fitx = 1*ploty**2 + 1*ploty

		ylist = np.concatenate((ylist,ploty))
		xlist = np.concatenate((xlist,left_fitx))

	points = np.vstack((xlist,ylist))
	# Translate points
	points = np.array([np.add(points[0], -720/2),np.add(points[1],-1280/2)])
	# Unrotate points
	theta = np.radians(90)
	R = np.array(((np.cos(theta), np.sin(theta)), (np.sin(theta),np.cos(theta))))
	points = np.matmul(R,points)
	# Untranslate points
	points = np.array([np.flip(np.add(points[0], 1280/2)),np.add(points[1],720/2)])

	return points

def draw_sliding_window(image):

	'''
	Crop to bottom half of image
	and create histogram of number of white pixels along each column
	'''
	crop_img = image[600:720,0:1280]
	histogram = np.sum(crop_img[:,:,0],axis = 0)

	# Define the threshold to apply to histogram
	mean = np.mean(histogram, axis = 0)
	std = np.std(histogram, axis = 0)
	cutoff = mean+std*3 - 150
	binsize = 20
	maxes = list()

	# plt.plot(histogram)
	# plt.plot([0, len(histogram)], [mean+std*2, mean+std*2], color='k', linestyle='-', linewidth=2)
	# plt.show()

	'''
	Find peaks in histogram that exceed cutoff,
	then record midpoint of bin, as well as the value of that peak
	'''
	for i in range(0,binsize):
		leftbound = int(i*1280/binsize)
		rightbound = int((i+1)*1280/binsize)
		midpoint = int((leftbound+rightbound)/2)
		binmax = np.amax(histogram[:][leftbound:rightbound],axis=0)
		if (binmax>cutoff):
			maxes.append([midpoint,binmax])

	# Hyperparameters
	# Choose the number of sliding windows
	nwindows = 20
	# Set the width of the windows +/- margin
	margin = 100
	# Set the minimum number of the pixels to recenter the windows
	minpix = 50
	# Set height of the windows - based on nwindows above and image shape
	window_height = image.shape[0] // nwindows

	xlist=np.empty(0)
	ylist=np.empty(0)

	# Identify the x and y positions of all nonzero (i.e. activated) pixels in the image
	nonzero = image.nonzero()
	nonzeroy = np.array(nonzero[0])
	nonzerox = np.array(nonzero[1])
	# Current positions to be updated later for each window in nwindows
	for i in range(0,len(maxes)):
		leftx_current = maxes[i][0] # Returns the midpoint of each x window

		# Create empty lists to receive left and right lane pixel indices
		left_lane_inds = []

		for window in range(nwindows):
			#Identify window boundaries in x and y
			win_y_low = image.shape[0] - (window+1)*window_height
			win_y_high = image.shape[0] - window*window_height
			win_xleft_low = leftx_current - margin
			win_xleft_high = leftx_current + margin

			# Draw the windows on the visualization image
			#cv2.rectangle(image,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2)

			# Identify the nonzero pixels in x and y within the window
			good_left_inds = ((nonzeroy >= win_y_low) \
							& (nonzeroy < win_y_high) \
							& (nonzerox >= win_xleft_low) \
							& (nonzerox < win_xleft_high)).nonzero()[0]
			# Append these indices to the lists
			left_lane_inds.append(good_left_inds)
			#if more than minpix pixels were found , recenter next window on their mean position
			if len(good_left_inds) > minpix:
				leftx_current = np.int(np.mean(nonzerox[good_left_inds]))

		# Concatenate the arrays of indices (previously was a list of lists of pixels)
		left_lane_inds = np.concatenate(left_lane_inds)
		# Extract left and right line pixel positions
		leftx = nonzerox[left_lane_inds]
		lefty = nonzeroy[left_lane_inds]

		# if(len(leftx) <= 5000):
		#     continue

		# Generate x and y values for plotting
		ploty = np.linspace(0, image.shape[0]-1, image.shape[0])
		try:
			# Fit a second order polynomial to each set of lane points
			left_fit = np.polyfit(lefty, leftx, 2)
			left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
		except TypeError:
			# Avoids an error if `left` and `right_fit` are still none or incorrect
			left_fitx = 1*ploty**2 + 1*ploty

		ylist = np.concatenate((ylist,ploty))
		xlist = np.concatenate((xlist,left_fitx))
		points = [xlist,ylist]

	return points

	# # Highlight the left and right lane regions
	# # drawing the left and right polynomials on the lane lines
	# pts_left = np.array([np.transpose(np.vstack([xlist, ylist]))])
	# # pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
	# # pts = np.hstack((pts_left, pts_right))

	# blank_image = np.zeros_like(image)
	# # Draw the lane onto a blank warped image
	# cv2.fillPoly(blank_image, np.int_([pts_left]), (0,255, 0))

	# # Transform the perspective back into the original view
	# unwarped_image = unwarp(blank_image)

	# # Add the unwarped processed image to the original image
	# weighted_image = cv2.addWeighted(image, 1.0, unwarped_image, .7, 0)
	# return weighted_image

		# Highlightign the left and right lane regions
		#image[lefty, leftx] = [255, 0, 0]


	#return image

# Draw lines originating from right, left and bottom of thresholded binary image
def highlight_all(image):
	points = []
	try:
		points = draw_sliding_window_left(image)
		# print( 'left points is ', points[0])
		plt.scatter(points[0],points[1],0.5,color='red')
	except:
		print("highlight_all: no lines from left")
	if(len(points) <= 2):
		try:
			points = draw_sliding_window(image)
			# print('points from bottom: ', points)
			plt.scatter(points[0],points[1],0.5,color='blue')
			print('points 0: ', points[0])
			print('points 1: ', points[1])
		except:
			print("highlight_all: no lines from bottom")
	# try:
	#     [x2,plotx_right]= draw_sliding_window_right(image)
	#     plt.scatter(plotx_right,720-x2, 0.5,color='yellow')
	# except:
	#     print("highlight_all: no lines from right")
	return image, points


## VIDEO PROCESSING-----------
# Highlights the lane in the original video stream with green rectangle
def highlight_lane_original(image):
	# Process the image to undistort, apply sat and sobel thresholding
	# and warp it into the bird's eye view
	processed_image = process_image(image, bboxes)

	# Crop to bottom half of image
	crop_img = processed_image[600:720,0:1280]
	histogram = np.sum(crop_img[:,:,0],axis = 0)

	# Find the peaks of the left and right halves of the histogram
	# as the starting point for the left and right lines
	midpoint = np.int(histogram.shape[0]//2)
	leftx_base = np.argmax(histogram[:midpoint])
	rightx_base = np.argmax(histogram[midpoint:])+midpoint

	# Hyperparameters
	# choose the number of sliding windows
	nwindows = 30
	# Set the width of the windows +/- margin
	margin = 50
	# Set the minimum number of the pixels to recenter the windows
	minpix = 50
	# Set height of the windows - based on nwindows above and image shape
	window_height = processed_image.shape[0] // nwindows

	# Identify the x and y positions of all nonzero (i.e. activated) pixels in the image
	nonzero = processed_image.nonzero()
	nonzeroy = np.array(nonzero[0])
	nonzerox = np.array(nonzero[1])
	# Current positions to be updated later for each window in nwindows
	leftx_current = leftx_base
	rightx_current = rightx_base

	# Create empty lists to receive left and right lane pixel indices
	left_lane_inds = []
	right_lane_inds = []
	for window in range(nwindows):
		#Identify window boundaries in x and y
		win_y_low = processed_image.shape[0] - (window+1)*window_height
		win_y_high = processed_image.shape[0] - window*window_height
		win_xleft_low = leftx_current - margin
		win_xleft_high = leftx_current + margin
		win_xright_low = rightx_current - margin
		win_xright_high = rightx_current + margin

		# Identify the nonzero pixels in x and y within the window
		good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
		good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
		# Append these indices to the lists
		left_lane_inds.append(good_left_inds)
		right_lane_inds.append(good_right_inds)
		#if more than minpix pixels were found , recenter next window on their mean position
		if len(good_left_inds) > minpix:
			leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
		if len(good_right_inds) > minpix:
			rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
	# Concatenate the arrays of indices (previously was a list of lists of pixels)
	left_lane_inds = np.concatenate(left_lane_inds)
	right_lane_inds = np.concatenate(right_lane_inds)

	# Extract left and right line pixel positions
	leftx = nonzerox[left_lane_inds]
	lefty = nonzeroy[left_lane_inds]
	rightx = nonzerox[right_lane_inds]
	righty = nonzeroy[right_lane_inds]


	# Generate x and y values for plotting
	ploty = np.linspace(0, processed_image.shape[0]-1, processed_image.shape[0])
	try:
		# Fit a second order polynomial to each set of lane points
		left_fit = np.polyfit(lefty, leftx, 2)
		right_fit = np.polyfit(righty, rightx, 2)
		left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
		right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
	except TypeError:
		# Avoids an error if `left` and `right_fit` are still none or incorrect
		left_fitx = 1*ploty**2 + 1*ploty
		right_fitx = 1*ploty**2 + 1*ploty

	# Highlight the left and right lane regions
	# drawing the left and right polynomials on the lane lines
	pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
	pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
	pts = np.hstack((pts_left, pts_right))

	blank_image = np.zeros_like(processed_image)
	# Draw the lane onto a blank warped image
	cv2.fillPoly(blank_image, np.int_([pts]), (0,255, 0))

	# Transform the perspective back into the original view
	unwarped_image = unwarp(blank_image)

	# Add the unwarped processed image to the original image
	weighted_image = cv2.addWeighted(image, 1.0, unwarped_image, .7, 0)
	return weighted_image

# Apply pipeline to all frames in input video and output
# video with green rectangle bounded by field lines
# def generate_video():
# 	clip1 = VideoFileClip("test_videos/solidWhiteRight.mp4").subclip(0,5)
# 	output_file_path = 'output_video.mp4'
# 	input_video = VideoFileClip("input_video.mp4")
# 	output_video = input_video.fl_image(highlight_lane_original) #NOTE: this function expects color images!!
# 	output_video.write_videofile(output_file_path, audio=False)

## TESTING-----------no worrie
# Test and visualize the vision pipeline on sample images
def detect_lanes():
	for i in range(0,10):
		image_name = str(i) + '.jpg'
		image = cv2.imread('input_images/'+'0.jpg')

		test_image = process_image(image, bboxes)
		sw, points = highlight_all(test_image)
		warped_image = warp(image)

		# Display resulting image
		# plt.title(i)
		# plt.imshow(warped_image)
		# plt.show()
		# plt.savefig('test/'+str(i)+'.png')
		# plt.close()

		# # generate ROS image
		# bridge = CvBridge()
		# imgMsg = bridge.cv2_to_imgmsg(sw, "bgr8")

if __name__ == '__main__':
	detect_lanes()
