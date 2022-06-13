import carla
from __init__ import client, world
from TB_common_functions import wraptopi, calculateDistance, AgentColourToRGB
# from path_planner_suite import astar_search, find_nearest

from shapely.geometry import LineString
import math
import numpy as np
import matplotlib.pyplot as plt

import collections
import weakref


from Agents.controller import long_lat_controller
from AVs_internal.frenet_optimal_trajectory import generate_target_course, frenet_optimal_planning

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

def get_actor_display_name(actor, truncate=250):
	name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
	return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================

class CollisionSensor(object):
	def __init__(self, parent_actor):
		self.sensor = None
		self.history = []
		self._parent = parent_actor
		world = self._parent.get_world()
		bp = world.get_blueprint_library().find('sensor.other.collision')
		self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
		# We need to pass the lambda a weak reference to self to avoid circular
		# reference.
		weak_self = weakref.ref(self)
		self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))


	def get_collision_history(self):
		history = collections.defaultdict(int)
		for frame, intensity in self.history:
			history[frame] += intensity
		return history

	@staticmethod
	def _on_collision(weak_self, event):
		self = weak_self()
		if not self:
			return
		actor_type = get_actor_display_name(event.other_actor)
		impulse = event.normal_impulse
		intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
		self.history.append((event.frame, intensity))
		if len(self.history) > 4000:
			self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================

class LaneInvasionSensor(object):
	def __init__(self, parent_actor):
		self.sensor = None
		self.invasion_flag = False

		# If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
		if parent_actor.type_id.startswith("vehicle."):
			self._parent = parent_actor
			world = self._parent.get_world()
			bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
			self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
			# We need to pass the lambda a weak reference to self to avoid circular
			# reference.
			weak_self = weakref.ref(self)
			self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

	@staticmethod
	def _on_invasion(weak_self, event):
		self = weak_self()
		if not self:
			self.invasion_flag = False
			return
		print("event.crossed_lane_markings",event.crossed_lane_markings)
		lane_types = set(x.type for x in event.crossed_lane_markings)
		print("lane_types",lane_types)
		text = ['%r' % str(x).split()[-1] for x in lane_types]
		# self.hud.notification('Crossed line %s' % ' and '.join(text))
		print("text = ",text)
		self.invasion_flag = True


# ==============================================================================
# -- ObstacleSensor ------------------------------------------------------------
# ==============================================================================

class ObstacleSensor(object):
	def __init__(self, parent_actor):
		self.sensor = None

		# If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
		if parent_actor.type_id.startswith("vehicle."):
			self._parent = parent_actor
			world = self._parent.get_world()
			self.bp = world.get_blueprint_library().find('sensor.other.obstacle')
			# self.bp.set_attribute('distance','5')
			# self.bp.set_attribute('hit_radius','5')
			self.bp.set_attribute('debug_linetrace','true')
			self.bp.set_attribute('only_dynamics', 'false')

	def spawn(self,x,y,z,yaw):
		self.sensor = world.spawn_actor(self.bp, carla.Transform(carla.Location(x=float(x), y=float(y), z=float(z)), carla.Rotation(yaw=float(yaw))), attach_to=self._parent)
		# We need to pass the lambda a weak reference to self to avoid circular
		# reference.
		weak_self = weakref.ref(self)
		self.sensor.listen(lambda event: ObstacleSensor._on_detection(weak_self, event))

	@staticmethod
	def _on_detection(weak_self, event):
		self = weak_self()
		if not self:
			return

		self.distance = event.distance
		print(self.distance)
		# self._event_count += 1
		# print ("Event %s, in line of sight with %s at distance %u" % (self._event_count, event.other_actor.type_id, event.distance))



		# print("event.crossed_lane_markings",event.crossed_lane_markings)
		# lane_types = set(x.type for x in event.crossed_lane_markings)
		# print("lane_types",lane_types)
		# text = ['%r' % str(x).split()[-1] for x in lane_types]
		# # self.hud.notification('Crossed line %s' % ' and '.join(text))
		# print("text = ",text)
		# self.invasion_flag = True


# ==============================================================================
# -- AutonomousVehicle --------------------------------------------------------
# ==============================================================================

class autonomous_vehicle():
	def __init__(self):
		self.execute_trigger_file = "TriggerScenarioExecution.csv"

		self.client = client
		self.world  = world

		self.blueprint_library = self.world.get_blueprint_library()
		self.vehicle_blueprint = self.blueprint_library.filter("vehicle.mercedes.coupe")[0]
		self.vehicle_blueprint.set_attribute('color', AgentColourToRGB("red"))

		self.STOPPING_DISTANCE = 5
		self.close_to_destination = False
		self.red_traffic_light    = False
		self.obstacle             = False

		self.stop  = False

		self.ignorePath = True

		self.vehicle_speed = 5 # Setting a default speed

		self.controller = long_lat_controller()
		
		self.frenet_start_flag = True

		self.current_collision = None
		self.last_collision = None


		# self.SpawnActor = carla.command.SpawnActor

	def spawn(self,x,y,z,yaw):
		self.actor_list = []
		self.spawn_orientation = yaw
		self.transform         = carla.Transform(carla.Location(x=float(x), y=float(y), z=float(z)), carla.Rotation(yaw=float(self.spawn_orientation)))
		
		self.agent           = self.world.spawn_actor(self.vehicle_blueprint, self.transform) 
		self.world.tick() # important to tick after spawning, otherwise actor details not reachable
		self.actor_list.append(self.agent)

		self.spawn_point_x   = x 
		self.spawn_point_y   = y 
		self.spawn_point_z   = z 
		self.spawn_point_yaw = yaw

		self.agentID    = self.agent.id
		self.agentType  = self.agent.type_id

		
		# Attaching sensors
		# self.obstacle_sensor_left_init_trans   = carla.Transform(carla.Location(z=1.5))
		# self.obstacle_sensor_centre_init_trans = carla.Transform(carla.Location(z=1.5))
		# self.obstacle_sensor_right_init_trans  = carla.Transform(carla.Location(z=1.5))
		# self.collision_sensor_init_trans       = carla.Transform(carla.Location(z=0.5))
		# self.lane_invasion_sensor_init_trans   = carla.Transform(carla.Location(z=0.5))


		# self.obstacle_sensor_left = world.spawn_actor(self.obstacle_sensor_bp, 
		#                                             self.obstacle_sensor_left_init_trans, 
		#                                             attach_to=self.agent)

		# self.obstacle_sensor_centre = self.world.spawn_actor(self.obstacle_sensor_bp, 
		#                                             self.obstacle_sensor_centre_init_trans, 
		#                                             attach_to=self.agent)

		# self.obstacle_sensor_right = world.spawn_actor(self.obstacle_sensor_bp, 
		#                                             self.obstacle_sensor_right_init_trans, 
		#                                             attach_to=self.agent)


		# self.collision_sensor = self.world.spawn_actor(self.collision_sensor_bp, 
		#                                             carla.Transform(), 
		#                                             attach_to=self.agent)

		# self.lane_invasion_sensor = world.spawn_actor(self.lane_invasion_sensor_bp, 
		#                                             self.lane_invasion_sensor_init_trans, 
		#                                             attach_to=self.agent)

		self.collision_sensor       = CollisionSensor(self.agent)

		self.lane_invasion_sensor   = LaneInvasionSensor(self.agent)
		
		self.obstacle_sensor_centre = ObstacleSensor(self.agent)
		self.obstacle_sensor_centre.bp.set_attribute('distance','5')
		self.obstacle_sensor_centre.spawn(0,0,0,0)

		self.obstacle_sensor_left   = ObstacleSensor(self.agent)
		self.obstacle_sensor_left.bp.set_attribute('distance','5')
		self.obstacle_sensor_left.spawn(0,0,0,-45)

		self.obstacle_sensor_right  = ObstacleSensor(self.agent)
		self.obstacle_sensor_right.bp.set_attribute('distance','5')
		self.obstacle_sensor_right.spawn(0,0,0,45)


		


	def set_destination(self,x,y,z):
		self.dest_x = x 
		self.dest_y = y 
		self.dest_z = z 

		self.PathPlan()

		print("Vehicle destination set!")

	def set_path(self,path_coord_tuple):

		self.path = path_coord_tuple  # [(x1,y1), (x1,y1), ...]


		
	def function_handler(self, event):
		actor_we_collide_against = event.other_actor
		impulse = event.normal_impulse
		intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)

	def step(self):
		print("==================")
		# # Check if exection trigger is on
		# with open(self.execute_trigger_file, 'r') as f:
		#   lines = csv.reader(f, delimiter=',', quotechar='|')
		#   rows = list(lines)

		# try:
		#   MoveToDestination = rows[0][0]
		# except:
		#   MoveToDestination = "False"

		# if MoveToDestination == "True":
		#   self.stop = True
		# else:
		#   self.stop = False

		# Get update of the world state
		self.world_snapshot = self.world.get_snapshot()

		self.Update_state_info()


		if self.stop == True:
			self.send_control(0, 0, 1, hand_brake=False, reverse=False)
		else:
			# self.send_control(self.throttle, 0, 1, hand_brake=False, reverse=False)
			self.get_to_destination()   
			# self.controller()

		# print(self.collision_sensor.get_collision_history()) #STOPPED HERE need to get a reading of the collision live
		
		

		# == Collision sensor update =======================
		# Check if there are any collsions
		if len(self.collision_sensor.history) == 0:
			print("No collsions yet.")

		else:
			# Get the current collision
			self.current_collision = self.collision_sensor.history[-1]

			# Check if current collision is the same as the last collision
			if self.current_collision == self.last_collision:
				print("There was collsion but AV continued moving")
			else:
				# If current collisiion and last collision are consecutive then it is the same collision but car stopped there else this is a new collision
				if self.last_collision == None or (self.last_collision[0]+1) == self.current_collision[0]:
					# print("self.last_collision[0] = ",self.last_collision[0])
					print("Same Collision -  AV stuck")

				else:
					print("New Collision after a pre exisitng collision")

		self.last_collision == self.current_collision


		# == Lane Invasion sensor update =====================
		if self.current_y <= 127.32:
			print("AV mounted left hand side kerb")
		elif self.current_y > 127.32 and self.current_y <= 131.44:
			print("AV is on correct lane")
		elif self.current_y > 131.44 and self.current_y <= 135.55:
			print("AV is on opposite lane")
		elif self.current_y > 135.55:
			print("AV mounted right hand side kerb")

		print("Invasion flag = ",self.lane_invasion_sensor.invasion_flag)
		self.lane_invasion_sensor.invasion_flag = False


		# == Obstacle sensor update =====================
		# self.obstacle_sensor


		

	def set_speed(self,speed):
		self.vehicle_speed = speed
		pass

	def Update_state_info(self):
		agent_transform                 = self.agent.get_transform()
		agent_location                  = agent_transform.location  
		agent_rotation                  = agent_transform.rotation
		agent_velocity                  = self.agent.get_velocity()      # This is an object vector

		self.current_t                  = self.world_snapshot.timestamp.elapsed_seconds #ts.elapsed_seconds - start_of_simulation_timestamp
		self.current_x                  = agent_location.x
		self.current_y                  = agent_location.y
		self.current_z                  = agent_location.z
		self.current_yaw                = wraptopi(math.radians(agent_rotation.yaw))
		self.current_velocity                = np.array([agent_velocity.x, agent_velocity.y, agent_velocity.z])      # This is an array as opposed to agent_velocity, which is an object
		self.current_speed              = np.sqrt(self.current_velocity.dot(self.current_velocity))


	def get_pos(self):
		self.Update_state_info()
		self.pos_array = [self.current_x, self.current_y, self.current_z, self.current_yaw]

		return self.pos_array

	def destroy(self):
		sensors = [
			self.collision_sensor.sensor,
			self.lane_invasion_sensor.sensor,
			self.obstacle_sensor_centre.sensor,
			self.obstacle_sensor_left.sensor,
			self.obstacle_sensor_right.sensor]

		for sensor in sensors:
			if sensor is not None:
				sensor.stop()
				sensor.destroy()

		self.agent.destroy()
		pass

	def get_actors_xy_pos(self):
		flag = False
		actors = self.world.get_actors()
		actors_pos = np.array([[]])
		for actor in actors:
			if ("vehicle" in actor.type_id) or ("walker.pedestrian" in actor.type_id):
				
				if actor.id == self.agent.id:
					continue # don't take pos of AV as one of the actors

				actor_location = actor.get_transform().location
				
				if flag == False:
					actors_pos = np.array([[actor_location.x,actor_location.y]])
					flag = True
				else:
					actors_pos = np.append(actors_pos,[[actor_location.x,actor_location.y]],axis=0)
		print("actors_pos = ",actors_pos)
		return actors_pos


	def get_to_destination(self):

		# Check if sensor detected and obstacle (Need to add the obstacles first)

		# If obstacle detected use frenet to recalcualte path


		# Send control
		throttle, brake, steer = self.controller.step(self.path, self.vehicle_speed, self.current_x, self.current_y, self.current_yaw, self.current_speed)
		self.send_control(throttle, steer, brake, hand_brake=False, reverse=False)

		# # update variables for frenet 
		# if self.frenet_start_flag:
		#   c_speed = 1  # current speed [m/s]
		#   c_d = 0 # current lateral position [m]
		#   c_d_d = 0.0  # current lateral speed [m/s]
		#   c_d_dd = 0.0  # current lateral acceleration [m/s]
		#   s0 = 0.0  # current course position

		#   self.frenet_start_flag = False
		# else:
		#   min_dist = float("inf")
		#   # print("len(self.path.s) = ",len(self.path.s))
		#   # print("len(self.path.x) = ",len(self.path.x))

		#   for i in range(len(self.path.x)):
		#       dist_AV_to_traj_point = calculateDistance(self.path.x[i], self.path.y[i], self.current_x, self.current_y)
		#       if dist_AV_to_traj_point < min_dist:
		#           min_dist = dist_AV_to_traj_point
		#           index = i

		#   s0 = self.path.s[index]
		#   c_d = self.path.d[index]
		#   c_d_d = self.path.d_d[index]
		#   c_d_dd = self.path.d_dd[index]
		#   c_speed = self.current_speed
		
		# # check c_speed < 1, frentet optimisor fails if c_speed<1
		# if c_speed < 1:
		#   c_speed = 1


		# # update actors positions
		# ob = self.get_actors_xy_pos()
		# print("ob = ", ob)
		# # ob = np.array([[31.5, -38.1]
  #                      # ])
	
		
		# # frenet_optimal_planning
		# self.newpath = frenet_optimal_planning(self.csp, s0, c_speed,c_d, c_d_d, c_d_dd, ob)
		# # print("newpath = ",self.newpath)
		# if self.newpath == None:
		#   self.path = self.path
		#   print("path = ",self.path)
		#   print("newpath = ",self.newpath)
		#   print("c_speed = ",c_speed)
		#   print("s0 = ",s0)
		#   print("c_d = ",c_d)
		#   print("c_d_d = ",c_d_d)
		#   print("c_d_dd = ",c_d_dd)

		# else:
		#   self.path = self.newpath

		
		# area = 20.0

		# # # print("path = ",self.path)
		# # plt.cla()
		# # # for stopping simulation with the esc key.
		# # plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
		# # plt.plot(self.tx, self.ty)
		# # if ob.size:
		# #     plt.plot(ob[:, 0], ob[:, 1], "xk")
		# # plt.plot(self.path.x[1:], self.path.y[1:], "-or")
		# # plt.plot(self.path.x[1], self.path.y[1], "vc")
		# # plt.xlim(self.path.x[1] - area, self.path.x[1] + area)
		# # plt.ylim(self.path.y[1] - area, self.path.y[1] + area)
		# # plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
		# # plt.grid(True)
		# # plt.pause(0.0001)

		# # print("self.path.x = ",self.path.x)
		# # print("self.path.y = ",self.path.y)
		# self.controller_path = []
		# for i in range(len(self.path.x)):
		#   self.controller_path.append((self.path.x[i],self.path.y[i]))
		# print("self.controller_path = ", self.controller_path)

		# # conrtoller step
		# throttle, brake, steer = self.controller.step(self.controller_path, self.vehicle_speed, self.current_x, self.current_y, self.current_yaw, self.current_speed)
		# # throttle, brake, steer = self.controller.step(self.AV_path, self.vehicle_speed, self.current_x, self.current_y, self.current_yaw, self.current_speed)


		# throttle_output      = np.fmax(np.fmin(throttle, 1.0), 0)
		# brake_output         = np.fmax(np.fmin(brake, 1.0), 0)
		# steer_output         = np.fmax(np.fmin(steer, 1.0), -1.0)

		# self.send_control(throttle_output, steer_output, brake_output, hand_brake=False, reverse=False)


	def PathPlan(self):
		# spawn_pos = [self.spawn_point_x,self.spawn_point_y]
		# destination_pos = [self.dest_x,self.dest_y] 
		# want_to_plot_path = False
		# want_to_plot = False
		# plot_every       = 10
		
		# xStart = spawn_pos_idx = find_nearest(self.map_x_coord, spawn_pos[0])[0][0]
		# yStart = spawn_pos_idy = find_nearest(self.map_y_coord, spawn_pos[1])[0][1]

		# xEnd = destination_pos_idx = find_nearest(self.map_x_coord, destination_pos[0])[0][0]
		# yEnd = destination_pos_idy =find_nearest(self.map_y_coord, destination_pos[1])[0][1]

		# start = (xStart, yStart)
		# end = (xEnd, yEnd)
		# cost = 1 # cost per movement

		# # Plot_map(AV_pos, Target_pos, map_objects, map_x_coord, map_y_coord)

		# path, path_mat = astar_search(start, end, cost, spawn_pos, destination_pos, self.occupancy_grid, self.map_x_coord, self.map_y_coord, want_to_plot,plot_every)

		# path_coord = []
		# for i in range(len(path)):
		#   coord_tuple = (self.map_x_coord[path[i]],self.map_y_coord[path[i]])
		#   path_coord.append(coord_tuple)
		# self.main_path = path_coord
		# self.path_shapely_line = LineString(self.main_path)
		# print(self.path_shapely_line)

		# wx = [x[0] for x in path_coord]
		# wy = [x[1] for x in path_coord]

		# TODO Interpolate here to create path of wx and wy
		interpolation_resolution_min = 1
		self.path = interpolate_path([(self.spawn_point_x, self.spawn_point_y),(self.dest_x,self.dest_y)], interpolation_resolution_min)
		wx = []
		wy = []
		for point in self.path:
			wx.append(point[0])
			wy.append(point[1])

		# print("AV_path = ", AV_path)
		# print("wx = ", wx)
		# print("wy = ", wy)
		# wx = [self.spawn_point_x, 29.4, self.dest_x]
		# wy = [self.spawn_point_y, -41.2,self.dest_y]
		# wx = [self.spawn_point_x, 29.4, 41.5]
		# wy = [self.spawn_point_y, -41.2,-47.4]

		self.tx, self.ty, self.tyaw, self.tc, self.csp = generate_target_course(wx, wy)
		# print("tx = ",self.tx)
		# print("ty = ",self.ty)
		# print("tyaw = ",self.tyaw)
		# print("tc = ",self.tc)
		# print("tcsp = ",self.tcsp)
		# if want_to_plot_path:
		#   self.Plot_path(spawn_pos, destination_pos, self.occupancy_grid, self.map_x_coord, self.map_y_coord, path_coord)

	def send_control(self, throttle, steer, brake, hand_brake=False, reverse=False):
		self.control = carla.VehicleControl()

		# Clamp all values within their limits
		steer                    = np.fmax(np.fmin(steer, 1.0), -1.0)
		throttle                 = np.fmax(np.fmin(throttle, 1.0), -1.0)
		brake                    = np.fmax(np.fmin(brake, 1.0), -1.0)

		self.control.steer       = steer
		self.control.throttle    = throttle
		self.control.brake       = brake
		self.control.hand_brake  = hand_brake
		self.control.reverse     = reverse

		self.agent.apply_control(self.control)
		

	# def Plot_path(self,AV_pos, Target_pos, map_objects, map_x_coord, map_y_coord, path_coord):
	#   for i in np.unique(map_objects):
	#       map_objects2 = np.array(map_objects)
	#       result = np.where(map_objects2 == i)
	#       x = map_x_coord[result[0],result[1]]
	#       y = map_y_coord[result[0],result[1]]

	#       if i == 1:
	#           chosenColour = "cornflowerblue"
	#           chosenLabel  = "North"
	#       elif i == 2:
	#           chosenColour = "royalblue"
	#           chosenLabel  = "South"
	#       elif i == 3:
	#           chosenColour = "darkmagenta"
	#           chosenLabel  = "East"
	#       elif i == 4:
	#           chosenColour = "magenta"
	#           chosenLabel  = "West"
	#       elif i == 5:
	#           chosenColour = "red"
	#           chosenLabel  = "North-East"
	#       elif i == 6:
	#           chosenColour = "pink"
	#           chosenLabel  = "North-West"
	#       elif i == 7:
	#           chosenColour = "green"
	#           chosenLabel  = "South-East"
	#       elif i == 8:
	#           chosenColour = "lime"
	#           chosenLabel  = "South-West"
	#       elif i == 9:
	#           chosenColour = "darkgoldenrod"
	#           chosenLabel  = "All Directions"
	#       elif i == 10:
	#           chosenColour = "lightgrey"
	#           chosenLabel  = "Curb"
	#       else:
	#           chosenColour = "whitesmoke"
	#           chosenLabel  = "Obstacle"

	#       plt.scatter(x, y, s=5, marker = "s", color = chosenColour,label=chosenLabel)
			
	#   x_val = [x[0] for x in path_coord]
	#   y_val = [x[1] for x in path_coord]
		
	#   plt.scatter(AV_pos[0],AV_pos[1], s=20, marker='x', color='black',label="Start")
	#   plt.scatter(Target_pos[0],Target_pos[1], s=20, marker='x', color='red',label="End")
	#   plt.plot(x_val,y_val,markersize=4,linestyle="--",color='white',label="path")
	#   plt.legend()
	#   # plt.draw()
	#   # plt.pause(3)
	#   plt.show()



# class traffic_light():


