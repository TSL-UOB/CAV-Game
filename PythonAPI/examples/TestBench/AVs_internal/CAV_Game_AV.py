import carla
from __init__ import client, world
from TB_common_functions import wraptopi, calculateDistance, AgentColourToRGB
# from path_planner_suite import astar_search, find_nearest

from shapely.geometry import LineString
import math
import numpy as np
import matplotlib.pyplot as plt

import csv

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
		# print("event.crossed_lane_markings",event.crossed_lane_markings)
		lane_types = set(x.type for x in event.crossed_lane_markings)
		# print("lane_types",lane_types)
		text = ['%r' % str(x).split()[-1] for x in lane_types]
		# self.hud.notification('Crossed line %s' % ' and '.join(text))
		# print("text = ",text)
		self.invasion_flag = True


# ==============================================================================
# -- ObstacleSensor ------------------------------------------------------------
# ==============================================================================

class ObstacleSensor(object):
	def __init__(self, parent_actor):
		self.sensor = None
		self.previous_timestamp = 0
		self.timestamp = 0

		# If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
		if parent_actor.type_id.startswith("vehicle."):
			self._parent = parent_actor
			world = self._parent.get_world()
			self.bp = world.get_blueprint_library().find('sensor.other.obstacle')
			# self.bp.set_attribute('distance','5')
			self.bp.set_attribute('hit_radius','0.75')
			self.bp.set_attribute('debug_linetrace','false')
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
		self.previous_timestamp = self.timestamp
		self.distance = event.distance
		self.timestamp = event.timestamp
		self.other_actor = event.other_actor
		self.obstacle_ahead = True
		# print("event.other_actor = ",event.other_actor)
		# print("event.other_actor.type_id = ",event.other_actor.type_id)
		# if self.distance > 0:
			
		# else:
			# self.obstacle_ahead = False

			# print("HERE=======================================")
			# print(self.distance)
			# print("HERE=======================================")
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
		self.execute_trigger_file = "../../../Unreal/CarlaUE4/Content/Carla/CAV_Game/Scripts/TriggerScenarioExecution.csv"
		self.score_file = "../../../Unreal/CarlaUE4/Content/Carla/CAV_Game/Scripts/Score.csv"

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
		self.frenet_flag = False

		self.current_collision = None
		self.last_collision = None

		self.obstacle_sensor_1_previous_timestamp        = 0
		self.obstacle_sensor_2_previous_timestamp        = 0
		self.obstacle_sensor_3_previous_timestamp        = 0
		self.obstacle_sensor_4_previous_timestamp        = 0
		self.obstacle_sensor_5_previous_timestamp        = 0
		self.obstacle_sensor_6_previous_timestamp        = 0
		self.obstacle_sensor_7_previous_timestamp        = 0
		self.obstacle_sensor_8_previous_timestamp        = 0
		self.obstacle_sensor_9_previous_timestamp        = 0
		self.obstacle_sensor_10_previous_timestamp        = 0
		self.obstacle_sensor_11_previous_timestamp        = 0

		self.obstacle_at_1        = False
		self.obstacle_at_2        = False
		self.obstacle_at_3        = False
		self.obstacle_at_4        = False
		self.obstacle_at_5        = False
		self.obstacle_at_6        = False
		self.obstacle_at_7        = False
		self.obstacle_at_8        = False
		self.obstacle_at_9        = False
		self.obstacle_at_10        = False
		self.obstacle_at_11        = False

		self.on_main_lane = True
		self.prev_on_main_lane_value = True
		self.transitioning_flag = False
		self.set_path_to_main_flag  = True 
		self.prev_set_path_to_main_flag = True
		self.last_timestamp_on_main_lane = 0
		self.continious_duration_on_opposite_lane = 0
		
		self.GameScore = 0 
		self.execution_started_flag = False


		# self.SpawnActor = carla.command.SpawnActor

	def spawn(self,x,y,z,yaw):
		self.GameScore = 0 
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

		self.collision_sensor       = CollisionSensor(self.agent)

		self.lane_invasion_sensor   = LaneInvasionSensor(self.agent)
		
		self.obstacle_sensor_1   = ObstacleSensor(self.agent)
		self.obstacle_sensor_1.bp.set_attribute('distance','4')
		self.obstacle_sensor_1.bp.set_attribute('hit_radius','1')
		# self.obstacle_sensor_1.bp.set_attribute('debug_linetrace','true')
		self.obstacle_sensor_1.spawn(0,0,1,-90)
		
		self.obstacle_sensor_2   = ObstacleSensor(self.agent)
		self.obstacle_sensor_2.bp.set_attribute('distance','4.5')
		self.obstacle_sensor_2.bp.set_attribute('hit_radius','1')
		# self.obstacle_sensor_2.bp.set_attribute('debug_linetrace','true')
		self.obstacle_sensor_2.spawn(0,0,1,-50)

		self.obstacle_sensor_3   = ObstacleSensor(self.agent)
		self.obstacle_sensor_3.bp.set_attribute('distance','6')
		self.obstacle_sensor_3.bp.set_attribute('hit_radius','1')
		# self.obstacle_sensor_3.bp.set_attribute('debug_linetrace','true')
		self.obstacle_sensor_3.spawn(0,0,1,-25)

		self.obstacle_sensor_4   = ObstacleSensor(self.agent)
		self.obstacle_sensor_4.bp.set_attribute('distance','8')
		self.obstacle_sensor_4.bp.set_attribute('hit_radius','1')
		# self.obstacle_sensor_4.bp.set_attribute('debug_linetrace','true')
		self.obstacle_sensor_4.spawn(0,0,1,-10)

		self.obstacle_sensor_5   = ObstacleSensor(self.agent)
		self.obstacle_sensor_5.bp.set_attribute('distance','10')
		self.obstacle_sensor_5.bp.set_attribute('hit_radius','1')
		# self.obstacle_sensor_5.bp.set_attribute('debug_linetrace','true')
		self.obstacle_sensor_5.spawn(0,0,1,0)

		self.obstacle_sensor_6   = ObstacleSensor(self.agent)
		self.obstacle_sensor_6.bp.set_attribute('distance','8')
		self.obstacle_sensor_6.bp.set_attribute('hit_radius','1')
		# self.obstacle_sensor_6.bp.set_attribute('debug_linetrace','true')
		self.obstacle_sensor_6.spawn(0,0,1,10)
		
		self.obstacle_sensor_7   = ObstacleSensor(self.agent)
		self.obstacle_sensor_7.bp.set_attribute('distance','6')
		self.obstacle_sensor_7.bp.set_attribute('hit_radius','1')
		# self.obstacle_sensor_7.bp.set_attribute('debug_linetrace','true')
		self.obstacle_sensor_7.spawn(0,0,1,25)

		self.obstacle_sensor_8   = ObstacleSensor(self.agent)
		self.obstacle_sensor_8.bp.set_attribute('distance','4.5')
		self.obstacle_sensor_8.bp.set_attribute('hit_radius','1')
		# self.obstacle_sensor_8.bp.set_attribute('debug_linetrace','true')
		self.obstacle_sensor_8.spawn(0,0,1,50)

		self.obstacle_sensor_9   = ObstacleSensor(self.agent)
		self.obstacle_sensor_9.bp.set_attribute('distance','4')
		self.obstacle_sensor_9.bp.set_attribute('hit_radius','1')
		# self.obstacle_sensor_9.bp.set_attribute('debug_linetrace','true')
		self.obstacle_sensor_9.spawn(0,0,1,90)

		self.obstacle_sensor_10   = ObstacleSensor(self.agent)
		self.obstacle_sensor_10.bp.set_attribute('distance','10.8')
		self.obstacle_sensor_10.bp.set_attribute('hit_radius','1')
		# self.obstacle_sensor_10.bp.set_attribute('debug_linetrace','true')
		self.obstacle_sensor_10.spawn(0,0,1,21.8)

		self.obstacle_sensor_11   = ObstacleSensor(self.agent)
		self.obstacle_sensor_11.bp.set_attribute('distance','10.8')
		self.obstacle_sensor_11.bp.set_attribute('hit_radius','1')
		# self.obstacle_sensor_11.bp.set_attribute('debug_linetrace','true')
		self.obstacle_sensor_11.spawn(0,0,1,-21.8)
		


	def set_destination(self,x,y,z):
		self.dest_x = x 
		self.dest_y = y 
		self.dest_z = z 

		interpolation_resolution_min = 1
		self.main_path     = interpolate_path([(self.spawn_point_x, self.spawn_point_y),(self.dest_x,self.dest_y)], interpolation_resolution_min)
		self.opposite_path = interpolate_path([(self.spawn_point_x, self.spawn_point_y+3.5),(self.dest_x,self.dest_y+3.5)], interpolation_resolution_min)
		self.path = self.main_path
		# wx = [self.spawn_point_x, 29.4, 41.5]
		# wy = [self.spawn_point_y, -41.2,-47.4]
		wx = [waypoint[0] for waypoint in self.main_path]
		wy = [waypoint[1] for waypoint in self.main_path]
		self.tx, self.ty, self.tyaw, self.tc, self.csp = generate_target_course(wx, wy)

		# self.PathPlan()

		print("Vehicle destination set!")

	def set_path(self,path_coord_tuple):

		self.main_path = path_coord_tuple  # [(x1,y1), (x1,y1), ...]
		self.path = self.main_path

		
	def function_handler(self, event):
		actor_we_collide_against = event.other_actor
		impulse = event.normal_impulse
		intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)

	def step(self):
		print("==================")

		# == Get update of the world state =================
		self.world_snapshot = self.world.get_snapshot()
		self.Update_state_info()


		# == Collision sensor update =======================
		# Check if there are any collsions
		if len(self.collision_sensor.history) == 0:
			print("No collsions yet.")
			self.AV_new_collision_flag             = False
			self.AV_stuck_in_collision_flag        = False
			self.AV_continued_after_collision_flag = False

		else:
			# Get the current collision
			self.current_collision = self.collision_sensor.history[-1]


			if self.last_collision == None:
				self.AV_new_collision_flag             = True
				self.AV_stuck_in_collision_flag        = False

			elif self.last_collision[0] == self.current_collision[0]:
				self.AV_new_collision_flag             = False
				self.AV_stuck_in_collision_flag        = False

			elif (self.last_collision[0]+10) >= self.current_collision[0]:
				self.AV_new_collision_flag             = False
				self.AV_stuck_in_collision_flag        = True

			elif (self.last_collision[0]+10) < self.current_collision[0]:
				self.AV_new_collision_flag             = True
				self.AV_stuck_in_collision_flag        = False

		# print("self.current_collision = ", self.current_collision)
		# print("self.last_collision = ", self.last_collision)
		self.last_collision = self.current_collision


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
		try:
			if not self.obstacle_sensor_1.timestamp == self.obstacle_sensor_1_previous_timestamp and self.obstacle_sensor_1.other_actor.type_id == "vehicle.tesla.model3" :
				if self.obstacle_sensor_1.other_actor.type_id == "vehicle.tesla.model3":
					self.obstacle_at_1 = True
					print("self.obstacle_at_1 = ", self.obstacle_at_1)
				else:
					self.obstacle_at_1 = False 
				self.obstacle_sensor_1_previous_timestamp = self.obstacle_sensor_1.timestamp
			else:
				self.obstacle_at_1 = False
		except:
			self.obstacle_at_1 = False

		try:
			if not self.obstacle_sensor_2.timestamp == self.obstacle_sensor_2_previous_timestamp and self.obstacle_sensor_2.other_actor.type_id == "vehicle.tesla.model3" :
				if self.obstacle_sensor_2.other_actor.type_id == "vehicle.tesla.model3":
					self.obstacle_at_2 = True
					print("self.obstacle_at_2 = ", self.obstacle_at_2)
				else:
					self.obstacle_at_2 = False
				self.obstacle_sensor_2_previous_timestamp = self.obstacle_sensor_2.timestamp
			else:
				self.obstacle_at_2 = False
		except:
			self.obstacle_at_2 = False

		try:
			if not self.obstacle_sensor_3.timestamp == self.obstacle_sensor_3_previous_timestamp and self.obstacle_sensor_3.other_actor.type_id == "vehicle.tesla.model3" :
				if self.obstacle_sensor_3.other_actor.type_id == "vehicle.tesla.model3":
					self.obstacle_at_3 = True
					print("self.obstacle_at_3 = ", self.obstacle_at_3)
				else:
					self.obstacle_at_3 = False
				self.obstacle_sensor_3_previous_timestamp = self.obstacle_sensor_3.timestamp
			else:
				self.obstacle_at_3 = False
		except:
			self.obstacle_at_3 = False

		try:
			if not self.obstacle_sensor_4.timestamp == self.obstacle_sensor_4_previous_timestamp: 
				if self.obstacle_sensor_4.other_actor.type_id == "vehicle.tesla.model3":
					self.obstacle_at_4 = True
					print("self.obstacle_at_4 = ", self.obstacle_at_4)
				else:
					self.obstacle_at_4 = False 
				self.obstacle_sensor_4_previous_timestamp = self.obstacle_sensor_4.timestamp
			else:
				self.obstacle_at_4 = False
		except:
			self.obstacle_at_4 = False

		try:
			if not self.obstacle_sensor_5.timestamp == self.obstacle_sensor_5_previous_timestamp:
				print("self.obstacle_sensor_5 = ", self.obstacle_sensor_5.other_actor.type_id)
				if self.obstacle_sensor_5.other_actor.type_id == "vehicle.tesla.model3":
					print("self.obstacle_at_5 = ", self.obstacle_at_5) 
					self.obstacle_at_5 = True 
				self.obstacle_sensor_5_previous_timestamp = self.obstacle_sensor_5.timestamp
			else:
				self.obstacle_at_5 = False
		except:
			self.obstacle_at_5 = False

		try:
			if not self.obstacle_sensor_6.timestamp == self.obstacle_sensor_6_previous_timestamp and self.obstacle_sensor_6.other_actor.type_id == "vehicle.tesla.model3" :
				if self.obstacle_sensor_6.other_actor.type_id == "vehicle.tesla.model3":
					self.obstacle_at_6 = True
					print("self.obstacle_at_6 = ", self.obstacle_at_6)
				else:
					self.obstacle_at_6 = False 
				self.obstacle_sensor_6_previous_timestamp = self.obstacle_sensor_6.timestamp
			else:
				self.obstacle_at_6 = False
		except:
			self.obstacle_at_6 = False

		try:
			if not self.obstacle_sensor_7.timestamp == self.obstacle_sensor_7_previous_timestamp and self.obstacle_sensor_7.other_actor.type_id == "vehicle.tesla.model3" :
				if self.obstacle_sensor_7.other_actor.type_id == "vehicle.tesla.model3":
					self.obstacle_at_7 = True
					print("self.obstacle_at_7 = ", self.obstacle_at_7)
				else:
					self.obstacle_at_7 = False 
				self.obstacle_sensor_7_previous_timestamp = self.obstacle_sensor_7.timestamp
			else:
				self.obstacle_at_7 = False
		except:
			self.obstacle_at_7 = False

		try:
			if not self.obstacle_sensor_8.timestamp == self.obstacle_sensor_8_previous_timestamp and self.obstacle_sensor_8.other_actor.type_id == "vehicle.tesla.model3" :
				if self.obstacle_sensor_8.other_actor.type_id == "vehicle.tesla.model3":
					self.obstacle_at_8 = True
					print("self.obstacle_at_8 = ", self.obstacle_at_8)
				else:
					self.obstacle_at_8 = False 
				self.obstacle_sensor_8_previous_timestamp = self.obstacle_sensor_8.timestamp
			else:
				self.obstacle_at_8 = False
		except:
			self.obstacle_at_8 = False

		try:
			if not self.obstacle_sensor_9.timestamp == self.obstacle_sensor_9_previous_timestamp and self.obstacle_sensor_9.other_actor.type_id == "vehicle.tesla.model3" :
				if self.obstacle_sensor_9.other_actor.type_id == "vehicle.tesla.model3":
					self.obstacle_at_9 = True
					print("self.obstacle_at_9 = ", self.obstacle_at_9)
				else:
					self.obstacle_at_9 = False 
				self.obstacle_sensor_9_previous_timestamp = self.obstacle_sensor_9.timestamp
			else:
				self.obstacle_at_9 = False
		except:
			self.obstacle_at_9 = False

		try:
			if not self.obstacle_sensor_10.timestamp == self.obstacle_sensor_10_previous_timestamp and self.obstacle_sensor_10.other_actor.type_id == "vehicle.tesla.model3" :
				if self.obstacle_sensor_10.other_actor.type_id == "vehicle.tesla.model3":
					self.obstacle_at_10 = True
					print("self.obstacle_at_10 = ", self.obstacle_at_10)
				else:
					self.obstacle_at_10 = False 
				self.obstacle_sensor_10_previous_timestamp = self.obstacle_sensor_10.timestamp
			else:
				self.obstacle_at_10 = False
		except:
			self.obstacle_at_10 = False



		# ==  Check if vehicle is on main lane  and whether it has reached it is set path================
		if self.set_path_to_main_flag == False and self.current_y < 133.0:
			self.on_main_lane = True
			self.transitioning_flag = True
			self.path = self.opposite_path

		elif self.set_path_to_main_flag == False and self.current_y >= 133.0:
			self.on_main_lane = False
			self.transitioning_flag = False
			self.path = self.opposite_path

		elif self.set_path_to_main_flag == True and self.current_y > 130.5:
			self.on_main_lane = False
			self.transitioning_flag = True
			self.path = self.main_path

		elif self.set_path_to_main_flag == True and self.current_y <= 130.5:
			self.on_main_lane = True
			self.transitioning_flag = False
			self.path = self.main_path

		if self.current_yaw > 0.1 or self.current_yaw < -0.1:
			self.transitioning_flag = True


		# == Check if set path change is not too quick
		# if self.set_path_to_main_flag == True and self.prev_set_path_to_main_flag == False:
		# 	self.last_change_path_timestamp = self.current_t
		# 	self.prev_set_path_to_main_flag = self.set_path_to_main_flag

		# elif self.set_path_to_main_flag == False and self.prev_set_path_to_main_flag == True:
		# 	self.last_change_path_timestamp = self.current_t
		# 	self.prev_set_path_to_main_flag = self.set_path_to_main_flag
		# print("self.last_change_path_timestamp = ",self.last_change_path_timestamp)
		# print("self.diff = ",self.current_t - self.last_change_path_timestamp)
		# if (self.current_t - self.last_change_path_timestamp) < 1:
		# 	print("I AM INNNNNNNNNNNNNNNNNNNNNNNNNNN")
		# 	if self.set_path_to_main_flag == True:
		# 		self.path = self.opposite_path

		# 	if self.set_path_to_main_flag == False:
		# 		self.path = self.main_path



		# == Check if exection button is on ================
		with open(self.execute_trigger_file, 'r') as f:
			lines = csv.reader(f, delimiter=',', quotechar='|')
			rows = list(lines)

		# print("list of vehicles = ",self.world.get_actors().filter("vehicle.tesla.model3"))

		try:
			ExecuteButtonFlag = rows[0][0]
		except:
			ExecuteButtonFlag = "False"

		if ExecuteButtonFlag == "True" and len(self.world.get_actors().filter("vehicle.tesla.model3")) > 0:
			
			if self.execution_started_flag == False:
				self.execution_start_time   = self.current_t
				self.execution_started_flag = True


		
			# == Behaviour tree =====================
			# print("self.current_y = ", self.current_y)
			# print("self.obstacle_sensor_5.other_actor.type_id == 'vehicle.tesla.model3' = ",self.obstacle_sensor_5.other_actor.type_id == "vehicle.tesla.model3")
			# print(self.obstacle_sensor_5.other_actor.type_id)
			# print("self.transitioning_flag = ", self.transitioning_flag)
			# print("self.on_main_lane = ", self.on_main_lane)
			# print("self.set_path_to_main_flag = ", self.set_path_to_main_flag)
			# self.stop = False

			if self.transitioning_flag:
				self.stop = False
				if self.set_path_to_main_flag == True:
					if any([self.obstacle_at_1, self.obstacle_at_2, self.obstacle_at_3, self.obstacle_at_11]): 
						self.path = self.opposite_path
						self.set_path_to_main_flag = False
						self.transitioning_flag = False

				if self.set_path_to_main_flag == False:
					if any([self.obstacle_at_7, self.obstacle_at_8, self.obstacle_at_9, self.obstacle_at_10]): 
						self.path = self.main_path
						self.set_path_to_main_flag = True
						self.transitioning_flag = False
					# self.set_path_to_main_flag = False
					# self.path = self.opposite_path

				# if self.set_path_to_main_flag == False and self.obstacle_at_10 == True:
				# 	self.set_path_to_main_flag = True
				# 	self.path = self.main_path



			elif self.on_main_lane:
				# Check opposite lane 
				# print("I am in zero")
				# print("self.obstacle_at_4 = ",self.obstacle_at_4)
				# print("self.obstacle_at_5 = ",self.obstacle_at_5)
				# print("self.obstacle_at_6 = ",self.obstacle_at_6)
				# print("any([self.obstacle_at_4, self.obstacle_at_5, self.obstacle_at_6]) = ",any([self.obstacle_at_4, self.obstacle_at_5, self.obstacle_at_6]))
				if any([self.obstacle_at_4, self.obstacle_at_5, self.obstacle_at_6]):
					# print("I am in one")
					if not any([self.obstacle_at_7, self.obstacle_at_8, self.obstacle_at_9, self.obstacle_at_10]):
						# print("I am in two")
						self.path = self.opposite_path
						self.set_path_to_main_flag = False
						self.stop = False
						# self.transitioning_flag = True
					else:
						self.path = self.main_path
						self.stop = True
				else:
					self.path = self.main_path
					self.stop = False
			
			elif not self.on_main_lane:
				# Check main lane
				if any([self.obstacle_at_1, self.obstacle_at_2, self.obstacle_at_3, self.obstacle_at_11]): 
					self.path = self.opposite_path
					self.set_path_to_main_flag = False
					self.stop = False
					if any([self.obstacle_at_4, self.obstacle_at_5, self.obstacle_at_6]):
						self.stop = True
				else :
					self.path = self.main_path
					self.set_path_to_main_flag = True
					self.stop = False


		else:
			self.stop = True
			self.execution_start_time = self.current_t

		# == Game Score =================================
		print("self.AV_new_collision_flag = ",self.AV_new_collision_flag)
		print("self.AV_stuck_in_collision_flag = ",self.AV_stuck_in_collision_flag)
		try:
			print("self.last_collision[0] = ",self.last_collision[0])
			print("self.current_collision[0] = ",self.current_collision[0])
		except:
			pass

		# Score for collision
		if self.AV_new_collision_flag == True and self.AV_stuck_in_collision_flag == False:
			self.GameScore += 1000

		if self.on_main_lane == True:
			# check when we were last not on main lane
			self.last_timestamp_on_main_lane = self.current_t

		# Score for staying in opposite lane for too long
		if self.on_main_lane == False:	
			self.continious_duration_on_opposite_lane = self.current_t - self.last_timestamp_on_main_lane
		else:
			self.continious_duration_on_opposite_lane = 0

		if self.continious_duration_on_opposite_lane > 3:
			if round(self.continious_duration_on_opposite_lane,1)%1 == 0:
				self.GameScore += 200

		# Score for delay in arrival
		if (self.current_t - self.execution_start_time) > 14:
			if round(self.current_t - self.execution_start_time,1)%1 == 0:
				self.GameScore += 50



		self.prev_on_main_lane_value = self.on_main_lane 
	
		f2 = open(self.score_file, "w")
		f2.write(str(self.GameScore))
		f2.close()




		# == Controls update ============================
		if self.stop == True:
			self.send_control(0, 0, 1, hand_brake=False, reverse=False)
		else:
			self.get_to_destination()




		

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
		self.current_velocity           = np.array([agent_velocity.x, agent_velocity.y, agent_velocity.z])      # This is an array as opposed to agent_velocity, which is an object
		self.current_speed              = np.sqrt(self.current_velocity.dot(self.current_velocity))


	def get_pos(self):
		self.Update_state_info()
		self.pos_array = [self.current_x, self.current_y, self.current_z, self.current_yaw]

		return self.pos_array

	def destroy(self):
		sensors = [
			self.collision_sensor.sensor,
			self.lane_invasion_sensor.sensor,
			self.obstacle_sensor_1.sensor,
			self.obstacle_sensor_2.sensor,
			self.obstacle_sensor_3.sensor,
			self.obstacle_sensor_4.sensor,
			self.obstacle_sensor_5.sensor,
			self.obstacle_sensor_6.sensor,
			self.obstacle_sensor_7.sensor,
			self.obstacle_sensor_8.sensor,
			self.obstacle_sensor_9.sensor,
			self.obstacle_sensor_10.sensor,
			self.obstacle_sensor_11.sensor]

		for sensor in sensors:
			if sensor is not None:
				sensor.stop()
				sensor.destroy()

		self.agent.destroy()
		pass

	def get_actors_xy_pos(self):
		flag = False
		actors = self.world.get_actors()
		# print("actors = ",actors)
		actors_pos = np.array([[]])
		for actor in actors:
			if ("vehicle.tesla.model3" in actor.type_id) or ("walker.pedestrian" in actor.type_id):
				
				if actor.id == self.agent.id:
					continue # don't take pos of AV as one of the actors

				actor_location = actor.get_transform().location
				
				if flag == False:
					actors_pos = np.array([[actor_location.x,actor_location.y]])
					flag = True
				else:
					actors_pos = np.append(actors_pos,[[actor_location.x,actor_location.y]],axis=0)
		# print("actors_pos = ",actors_pos)
		return actors_pos


	def get_to_destination(self):

		# # If obstacle detected use frenet to recalcualte path
		# if self.frenet_flag:
		# 	print("I am path planning")
		self.PathPlan()
		# else:
		# 	print("I am not path planning")
		# 	self.path = self.main_path

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
		pass

		# # Check which obstacle sensor is on. 
		# if front obstacle sensor is off:
		# 	self.path = self.main_path
		# 	self.on_main_lane = True

		# elif front obsacle is on and right hand side obstacle sensor and right-front obstacle sensors are off:
		# 	self.path = self.opposite_path
		# 	self.on_main_lane = False

		# if not self.on_main_lane and left hand side obstacle sensor and left-front obstacle sensors are off:
		# 	self.path = self.main_path
		# 	self.on_main_lane = True




		# #################################################################
		# wx = [x[0] for x in path_coord]
		# wy = [x[1] for x in path_coord]

		# self.tx, self.ty, self.tyaw, self.tc, self.csp = generate_target_course(wx, wy)

		# # Update actors positions
		# ob = self.get_actors_xy_pos()
		# print("ob = ", ob)

		# # Update variables for frenet 
		# if self.frenet_start_flag:
		# 	s0 = 0.0  # current course position
		# 	c_d = 0 # current lateral position [m]
		# 	c_d_d = 0.0  # current lateral speed [m/s]
		# 	c_d_dd = 0.0  # current lateral acceleration [m/s]
		# 	c_speed = 5  # current speed [m/s]

		# 	self.frenet_start_flag = False
		# 	print("===================================================")
		# 	print("I am hereeeeeeeeeeeeeeeeeeeeeeeeeeeee!!!!!!!!!!")
		# 	print("===================================================")
		# else:
		# 	min_dist = float("inf")
		# 	# print("len(self.path.s) = ",len(self.path.s))
		# 	# print("len(self.path.x) = ",len(self.path.x))

		# 	for i in range(len(self.frenet_path.x)):
		# 		dist_AV_to_traj_point = calculateDistance(self.frenet_path.x[i], self.frenet_path.y[i], self.current_x, self.current_y)
		# 		if dist_AV_to_traj_point < min_dist:
		# 			min_dist = dist_AV_to_traj_point           
		# 			index = i

		# 	s0 = self.frenet_path.s[index]
		# 	c_d = self.frenet_path.d[index]
		# 	c_d_d = self.frenet_path.d_d[index]
		# 	c_d_dd = self.frenet_path.d_dd[index]
		# 	c_speed = self.current_speed

		# # check c_speed < 1, frentet optimisor fails if c_speed<1
		# if c_speed < 1:
		# 	c_speed = 1

		# print("==================")
		# print("s0 = ", s0)
		# print("==================")
		# # frenet_optimal_planning
		# self.frenet_path = frenet_optimal_planning(self.csp, s0, c_speed,c_d, c_d_d, c_d_dd, ob)
		# print("frenet_path = ",self.frenet_path)
		# # print("newpath = ",self.newpath)
		# if self.frenet_path == None:
		# 	self.path = self.path
		# 	# print("path = ",self.path)
		# 	# print("newpath = ",self.newpath)
		# 	# print("c_speed = ",c_speed)
		# 	# print("s0 = ",s0)
		# 	# print("c_d = ",c_d)
		# 	# print("c_d_d = ",c_d_d)
		# 	# print("c_d_dd = ",c_d_dd)

		# else:
		# 	self.path = []
		# 	x = []
		# 	y = []
		# 	for temp_i in range(len(self.frenet_path.x)):
		# 		self.path.append((self.frenet_path.x[temp_i],self.frenet_path.y[temp_i])) 
		# 		x.append(self.frenet_path.x[temp_i]) 
		# 		y.append(self.frenet_path.y[temp_i]) 

		# plt.plot(x,y)
		# plt.savefig("temp.png")
		

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
		


