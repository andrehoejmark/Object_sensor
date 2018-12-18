import math
import numpy as np


def find_object_width(x1, x2, array_degrees, array_distances):
	x1_deg = pixels_to_degrees(x1)
	x2_deg = pixels_to_degrees(x2)
	return match_with_array(x1_deg, x2_deg, array_degrees, array_distances)
	

def pixels_to_degrees(pixels):
	return pixels / (640 / 62) #pixel width / one deg pixlar


def match_with_array(deg_to_x1, deg_to_x2, array_degrees, array_distances):
	v = deg_to_x2 - deg_to_x1
	indexes = np.where((array_degrees > (v-5))*(array_degrees < (v+5)))
	
	lowest_distance = 100
	for i in range(len(indexes[0])):
		#print('i' + str(i))
		
		
		dist = array_distances[indexes[0][i]]
		if dist < lowest_distance:
			lowest_distance = dist
	
	
	

	object_width = math.sqrt(lowest_distance**2 + lowest_distance**2 - 2*lowest_distance*lowest_distance*math.cos(math.radians(v)))
	
	return object_width
	
	
	

array_degrees = np.array([45, 50, 55, 60, 65, 70])
array_distances = np.array([10, 12, 14, 16, 18, 20])

print(find_object_width(x1, x2, array_degrees, array_distances)
	
