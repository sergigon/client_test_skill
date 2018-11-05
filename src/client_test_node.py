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
import client_test_skill.msg ############# Cambiar esto #############

pkg_name = 'client_test_skill' ############# Cambiar esto #############
roslib.load_manifest(pkg_name)

server_name = 'test_skill'

# declare this only if the name is different of 'pkg_name'
skill_name = "client_test_skill" ############# Cambiar esto #############

### rospy.loginfo("Subscriber: " + msg.data + str(self.__counter))

class ClientTestSkill(Skill): ############# Cambiar esto #############

	#_feedback = client_test_skill.msg.TestFeedback() ############# Cambiar esto #############
	#_result = client_test_skill.msg.TestResult() ############# Cambiar esto #############

	def __init__(self):
		"""
		Init method.
		"""
		# init the skill
		Skill.__init__(self, skill_name, CONDITIONAL)

		# class variables
		self._client = None
		self._goal = 0
		self._out = False

	def create_msg_srv(self):
		"""
		This function has to be implemented in the children.
 		"""
 		print("create_msg_srv")
		# publishers and subscribers
		# servers and clients

		# actions
		action_module = self.my_import(test_skill.msg.testAction, testAction)
		self._client = actionlib.SimpleActionClient(server_name, action_module) ############# Cambiar esto #############

		self._client.wait_for_server()

		#Create the goal:

		'''
		if not self._as:

			self._as = actionlib.SimpleActionClient(pkg_name, client_test_skill.msg.ClientAction) ############# Cambiar esto #############

			# start the action server
			self._as.start()
		'''

	def shutdown_msg_srv(self):
		"""
		This function has to be implemented in the children.
		"""

		# publishers and subscribers
		# FIXME: do not unregister publishers because a bug in ROS
		# self.__test_pub.unregister()

		# servers and clients

	def my_import(module_name, class_name):
		"""
		Function to import a module from a string.

		@param module_name: The name of the module.
		@param class_name: The name of the class.
		"""

		# load the module, will raise ImportError if module cannot be loaded
		m = importlib.import_module(module_name)

		# get the class, will raise AttributeError if class cannot be found
		c = getattr(m, class_name)

		return c


	def execute_cb(self, goal):
		"""
		Spinner of the node.
		"""
		rospy.loginfo('[TEST SKILL]') ############# Cambiar esto #############
	
		# default values (SUCCESS)
		self._result.result = False
		self._feedback.feedback = 0

		while not self._out:
			# sleep 2 seconds
			rospy.sleep(2.0)
			
			print("self._status: "+ str(self._status) + ", self.RUNNING:" + str(self.RUNNING))
			if self._status == self.RUNNING:
				print ("Running...")
				try:
					self._counter = self._counter + 1
					print("self._counter: " + str(self._counter))
					if self._counter >= 20:
						self._result.result = True
						self._out = True

					if self._as.is_preempt_requested():
						print("Preempt requested")
						raise ActionlibException

				except ActionlibException:
					
					rospy.logwarn('[%s] Preempted or cancelled' % pkg_name)                 
					# FAIL
					self._result.result = 1
					self._feedback.feedback = 0
					

			else:
				rospy.logwarn("[%s] Cannot send a goal when the skill is stopped" % pkg_name)
				# ERROR
				self._result.result = -1
				self._feedback.feedback = 0


			self._feedback.feedback = self._counter
			self._as.publish_feedback(self._feedback)

		print("Fuera del while")

		if self._result.result:
			self._as.set_succeeded(self._result)
		else:
			rospy.logdebug("setting goal to preempted")
			self._as.set_preempted(self._result)


		#Inicializacion variables
		self._counter = 0
		self._out = False



if __name__ == '__main__':

	try:
		# start the node
		rospy.init_node(skill_name)

		# create and spin the node
		node = ClientTestSkill() ############# Cambiar esto #############
		rospy.spin()

	except rospy.ROSInterruptException:
		pass


		'''
		# Akinator
			self._feedback.progression = self.progression
			self._as.publish_feedback(self._feedback)

		if self._result.result:
			self._as.set_succeeded(self._result)
		else:
			rospy.logdebug("setting goal to preempted")
			self._as.set_preempted(self._result)
			'''