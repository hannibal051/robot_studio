import pybullet as p
import pybullet_data as pd
import time
import numpy as np
import random
import math

class Cheetah(object):
	def __init__(self):
		# self.pybullet_client = self._pybullet_client = bc.BulletClient(connection_mode=pybullet.GUI)
		self.generation = 100
		self.pybullet_client = p
		self.pybullet_client.connect(self.pybullet_client.GUI)
		self.pybullet_client.setAdditionalSearchPath(pd.getDataPath())
		self.motor_id_list = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
		self.pybullet_client.setGravity(0, 0, -9.8)
		# self.upper_angle = self.pybullet_client.addUserDebugParameter("upper_angle", 0, 1, 0.4)
		self.pybullet_client.resetDebugVisualizerCamera(3, 45, -30, [1, -1, 1])
		self.reset()

	def reset(self):
		init_position = [0, 0, 2]
		self._ground_id = self.pybullet_client.loadURDF('plane.urdf')
		self.quadruped = self.pybullet_client.loadURDF(
			"C:/my_stuff/my_py_project/untitled/bullet3-master/examples/pybullet/gym/pybullet_data/mini_cheetah/cookie.urdf",
			init_position,
			useFixedBase=False)

		num_joints = self.pybullet_client.getNumJoints(self.quadruped)
		for i in range(num_joints):
			print(self.pybullet_client.getJointInfo(self.quadruped, i))
		for i in range(4):
			self.reset_pos(i, 0.7853982)

	def step(self):
		a = [0.05,0.2,0.1,0.05,0.3,0.1,0.05,0.2,0.1,0.05,0.3,0.1]
		## b = [0.1,0.5,0.2,0.1,0.45,0.2,0.1,0.5,0.2,0.1,0.4,0.2]
		k = 0
		displacement = 0
		while k<=self.generation:
			t = 0
			pos0, orn0 = p.getBasePositionAndOrientation(self.quadruped)
			while t<=20:
				t += 0.001
				# angle = self.pybullet_client.readUserDebugParameter(self.upper_angle)
				for i in range(12):
					angle = a[i] * np.sin(2*t+0.5)
					self.reset_pos(i, angle)
				self.pybullet_client.stepSimulation()
				pos1, orn1 = p.getBasePositionAndOrientation(self.quadruped)
			dpos = math.sqrt(math.pow(pos1[0]-pos0[0],2) + math.pow(pos1[1]-pos0[1],2) + math.pow(pos1[2]-pos0[2],2))
			if dpos <= displacement:
				for i in range(12):
					a[i] = 0.5 * random.random()
			else:
				displacement = dpos
			k += 1
			self.pybullet_client.removeBody(self.quadruped)
			self.reset()

	def reset_pos(self, leg_id, angle):
		self.pybullet_client.setJointMotorControl2(self.quadruped,
												   jointIndex=self.motor_id_list[leg_id],
												   controlMode=self.pybullet_client.POSITION_CONTROL,
												   targetPosition=angle)


if __name__ == '__main__':
    env = Cheetah()
    env.step()