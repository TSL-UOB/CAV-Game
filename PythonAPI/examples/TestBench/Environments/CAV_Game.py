# from AVs_internal.AV_UoB import autonomous_vehicle
# from path_planner_suite import map_section, Generate_Numeric_Map
import random
import numpy as np

# from shapely.geometry import Point
# from shapely.geometry import Polygon

from AVs_internal.CAV_Game_AV import autonomous_vehicle
from Agents.vehicle import vehicle

# Contains 
#  Spawn point
#  Destination
#  Speed
#  Paths of lanes AV can use
def interpolate_path(path_tuple, increments_resolution):
	
	path_tuple_interpolated = []
	
	path_tuple_interpolated.append(path_tuple[0])
	
	for i in range(len(path_tuple)-1):
		
		diff = abs(np.subtract(path_tuple[i], path_tuple[i+1]))
		dist = np.hypot(diff[0],diff[1])
		
		if dist > increments_resolution:

			num_points_remainder = dist%increments_resolution
			num_points = int(np.ceil((dist - num_points_remainder)/increments_resolution)) + 1
			
			if num_points_remainder > 0:
				num_points = num_points + 1 
				
			x = np.linspace(path_tuple[i][0], path_tuple[i+1][0], num_points)
			x = x[1:]
			x = [round(num, 1) for num in x]

			y = np.linspace(path_tuple[i][1], path_tuple[i+1][1], num_points)
			y = y[1:]
			y = [round(num, 1) for num in y]

			interolated_points = list(zip(x, y))
			path_tuple_interpolated = path_tuple_interpolated + interolated_points
			# print(path_tuple_interpolated)
			
	return(path_tuple_interpolated)

class AV_on_Stright_Road():
	def __init__(self, world):
		self.spawn_height = 1
		self.world = world


	def set(self):
		self.AV = autonomous_vehicle()
		# self.AV = vehicle(1)

		x_spawn = 119.46
		y_spawn = 129.75
		z_spawn = 0.30
		yaw_spawn = 0

		# self.AV.spawn(x_spawn,y_spawn,z_spawn,yaw_spawn)
		self.AV.spawn(x_spawn, y_spawn, z_spawn, yaw_spawn)


		x_dest = 176.58
		y_dest = 129.74
		z_dest = 0.30 
		# self.AV.set_destination(x_dest,y_dest,z_dest)
		# self.AV_path = [(119.46, 129.75), (176.58, 129.74)]
		# self.AV_path = [(119.46, 129.75), (156.58, 145.74), (176.58, 129.74)]
		self.AV_path = [(119.46, 129.75), (136.58, 132.74), (155.58, 129.74), (176.58, 129.74)]
		interpolation_resolution_min = 1
		self.AV_path_interpolated  = interpolate_path(self.AV_path, interpolation_resolution_min)
		self.AV.set_path(self.AV_path_interpolated)

		self.AV.vehicle_speed = 5

	# 	self.actors_list = []
	# 	self.veh1 = vehicle(1)
	# 	self.veh2 = vehicle(2)
	# 	self.veh3 = vehicle(3)

		# # spawn(self,x,y,z,yaw):
		# # Setting Veh 1
		# # self.veh1_path = [[-6.44,-42.19],[-2.94,-61.59],[-6.44,-79.05]]
		# self.veh1_path = [(-368.2,209.8),(-342.6,197.1),(-334.4,197.1),(-317.9,189.1),(-314.5,183.3),(-280.1,166.3)]
		# self.veh2_path = [[-328.4,190.3]]
		# # self.veh3_path = [(-242.3,151.6),(-365.6,212.7)] # Safe Trajectory
		# # self.veh3_path = [(-258.6,159.4),(-365.6,212.7)] # Near miss Trajectory
		# # self.veh3_path = [(-279.0,169.4),(-365.6,212.7)] # Crash Trajectory
		# # self.veh3_path = [(-302.0,181.4),(-365.6,212.7)] # Ending Interolation Trajectory
		# if self.i == 0:
		# 	# Creating several paths for contious runs
		# 	veh3_path_series_start = [-242.3, 151.6] # Start of Trajectory series for vehicle with varying starting point
		# 	veh3_path_series_end = [-322.6, 191.2] # End of Trajectory series for vehicle with varying starting point
		# 	step = 5
		# 	self.timeout_thresh = 32
		# 	x = [-veh3_path_series_start[0], -veh3_path_series_end[0]]
		# 	x_new = np.arange(x[0],x[1],step)
		# 	y = [veh3_path_series_start[1], veh3_path_series_end[1]]
		# 	self.y_new = np.interp(x_new, x, y)
		# 	self.y_new = np.append(self.y_new,veh3_path_series_end[1])
		# 	self.x_new = x_new * -1
		# 	self.x_new = np.append(self.x_new,veh3_path_series_end[0])

		
		# self.veh3_path = [(self.x_new[self.i],self.y_new[self.i]),(-365.6,212.7)] 
		# self.i = self.i + 1
		# if self.i == len(self.x_new):
		# 	self.tests_ended = True


		# # interpolate path(s)
		# interpolation_resolution_min = 1
		# self.veh1_path_interpolated  = interpolate_path(self.veh1_path, interpolation_resolution_min)
		# self.veh2_path_interpolated  = interpolate_path(self.veh2_path, interpolation_resolution_min)
		# self.veh3_path_interpolated  = interpolate_path(self.veh3_path, interpolation_resolution_min)

		# self.veh1.set_path(self.veh1_path_interpolated)
		# self.veh2.set_path(self.veh3_path_interpolated)
		# self.veh3.set_path(self.veh3_path_interpolated)

		# self.veh1.spawn(self.veh1_path[0][0],self.veh1_path[0][1], self.spawn_height, -25)
		# self.veh2.spawn(self.veh2_path[0][0],self.veh2_path[0][1], self.spawn_height, -25)
		# self.veh3.spawn(self.veh3_path[0][0],self.veh3_path[0][1], self.spawn_height, 155)

		# self.veh1_speed = 5
		# self.veh2_speed = 0
		# self.veh3_speed = 5


	def step(self):
		self.AV.step() 

	# 	self.veh2.step() 
	# 	self.veh3.step()

	# 	self.veh2.stop = True



	# 	# Log 
	# 	t   = self.world.get_snapshot().timestamp.elapsed_seconds
	# 	fps = 1 / (self.world.get_snapshot().timestamp.frame_count)
	# 	self.tests_logs.append(0,0,self.actors_list,t,fps)
	# 	if self.tests_logs.time_array[-1] >= self.timeout_thresh:
	# 		self.test_ended = True



	# 	pass

	def destroy(self):
		self.AV.destroy()
		# self.tests_logs.write_file("Overtaking_TestID"+str(self.i)+".txt")
		# self.tests_logs.write_file("AssertionCheckingCaseStudyLogs_NearMiss.txt")
		# self.tests_logs.write_file("AssertionCheckingCaseStudyLogs_Crash.txt")
	
