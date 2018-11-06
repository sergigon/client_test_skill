#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Elena Velázquez and Sergio González"
__copyright__ = "Social Robots Group. Robotics Lab. University Carlos III of Madrid"
__credits__ = ["Elena Velázquez and Sergio González"]
__license__ = "LEUC3M v1.0"
__version__ = "0.0.0"
__maintainer__ = "Elena Velázquez and Sergio González"
__email__ = "serigon@ing.uc3m.es"
__status__ = "Development"

from skill.skill import Skill, ActionlibException, CONDITIONAL
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty

import roslib
import importlib
import rospy
import actionlib
import multimedia_msgs.msg

pkg_name = 'client_test_skill'
roslib.load_manifest(pkg_name)

server_name = 'test_skill'


class Client():
	
	def __init__(self):
		"""
		Init method.
		"""
		
		# class variables
		
		self._client = None
		self._result = None
		self._client_handle = None
		self._goal = None
		
		# Creates the ActionClient
		print("client_test_skill creating client")
		self._client = actionlib.ActionClient(server_name, multimedia_msgs.msg.TestAction)
		# Waits for server
		print("waiting for server")
		self._client.wait_for_server()

		# Creates the goal
		text = "hola te mando un goal desde el cliente " + pkg_name
		self._goal = multimedia_msgs.msg.TestGoal(text)
		# Sends the goal to the action server
		self._client_handle = self._client.send_goal(self._goal)
		print(pkg_name + " sending goal")
		

		if self._client_handle != None:
			rospy.sleep(10.0)
			# wait for the server to finish performing the action
			cancel=False
			once = False
			while self._client_handle.get_goal_status() != 2 and self._client_handle.get_goal_status()!=3 and self._client_handle.get_goal_status() != 4:
				if cancel and not once:
					rospy.logwarn("Canceled!")
					once = True
					# cancel current goal
					self._client_handle.cancel()
				print(pkg_name + " waiting for finish")
				rospy.sleep(2.0)

			# sleep some time to wait the response
			rospy.sleep(1.0)
			self._result = self._client_handle.get_result()
		else:
			print("No hay servidor")
			

if __name__ == '__main__':

	try:
		# start the node
		rospy.init_node(pkg_name)
		
		node = Client()

	except rospy.ROSInterruptException:
		pass
