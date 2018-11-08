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

# declare this only if the name is different of 'pkg_name'
skill_name = "client_server_test_skill"

server_name = 'test_skill'

# Result:
# -1: Si ha habido algun error o cancelacion
# 1: Si todo va bien


class ClientServerTestSkill(Skill):

	# Feedback and result of this skill
	_feedback = multimedia_msgs.msg.ClientFeedback()
	_result = multimedia_msgs.msg.ClientResult()

	def __init__(self):
		"""
		Init method.
		"""
		# init the skill
		Skill.__init__(self, skill_name, CONDITIONAL)

		# class variables
		self._as = None # SimpleActionServer variable
		self._goal = 0 # Goal a recibir
		
		self._sub_stop = None # Subscriber to external stop
		
		self._out = False # Variable de salida del loop en el execute_cb
		self._counter = 0 
		
		#self._client = None

	def create_msg_srv(self):
		"""
		This function has to be implemented in the children.
 		"""
 		print("create_msg_srv() called")
		# publishers and subscribers
		#self._sub_stop = rospy.Subscriber("chatter", String, callback)
		
		# servers and clients

		# actions
		# action_module = self.my_import(test_skill.msg.testAction, testAction)
		#self._client = actionlib.SimpleActionClient(server_name, action_module)

		#self._client.wait_for_server()

		# Si el servidor actionlib no se ha inicializado:
		if not self._as:
			self._as = actionlib.SimpleActionServer(skill_name, multimedia_msgs.msg.ClientAction, execute_cb=self.execute_cb, auto_start=False)

			# start the action server
			self._as.start()
		

	def shutdown_msg_srv(self):
		"""
		This function has to be implemented in the children.
		"""

		# publishers and subscribers
		# FIXME: do not unregister publishers because a bug in ROS
		# self.__test_pub.unregister()

		# servers and clients
		
		print("shutdown_msg_srv() called")

	'''
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
	'''

	def execute_cb(self, goal): # Se activa cuando le envias un goal a la skill
		"""
		Spinner of the node.
		"""
		rospy.loginfo('[' + pkg_name + ']')
	
		# default values (SUCCESS)
		self._result.result = False
		self._feedback.feedback = 0

		################### Loop de ejecucion ##########################
		while not self._out: # Sale del loop cuando self._out esta a True
			# sleep 2 seconds
			rospy.sleep(2.0)
			
			############### Si la skill esta activa: ###################
			if self._status == self.RUNNING:
				print ("RUNNING...")
				try:
					# Si el goal esta en estado Preempted (es decir, hay
					# un goal en cola o se cancela el goal actual),
					# activo la excepcion ##############################
					if self._as.is_preempt_requested():
						print("Preempt requested")
						raise ActionlibException
					#==================================================#
					
					################ Proceso el goal ###################
					if self._counter == 0:
						if  goal.command != '':
							rospy.loginfo("goal:" + str(goal.command))
						else:
							rospy.loginfo("goal vacio")
					self._counter = self._counter + 1
					print("self._counter: " + str(self._counter))
					if self._counter >= 30:
						self._result.result = 1
						self._out = True
					#==================================================#
				
				######### Si se ha hecho un preempted o cancel: ########
				except ActionlibException:
					rospy.logwarn('[%s] Preempted or cancelled' % pkg_name)                 
					# FAIL
					self._result.result = -1
					self._feedback.feedback = -1
					self._out = True # Salgo del loop
				#======================================================#
				
			#==========================================================#
	
			############# Si la skill no esta activa: ##################
			else:
				print("STOPPED")
				rospy.logwarn("[%s] Cannot send a goal when the skill is stopped" % pkg_name)
				# ERROR
				self._result.result = -1
				self._feedback.feedback = -1
				self._out = True # Salgo del loop
			#==========================================================#
			
			# Envio el feedback al final del loop
			self._feedback.feedback = self._counter
			self._as.publish_feedback(self._feedback)
		#================ Acaba el loop de ejecucion ==================#

		print("Fuera del while")
		
		#### Envio del resultado y actualizacion del status del goal ###
		print("Envío resultado")
		if self._result.result:
			rospy.logdebug("setting goal to succeeded")
			self._as.set_succeeded(self._result)
		else:
			rospy.logdebug("setting goal to preempted")
			self._as.set_preempted(self._result)
		#==============================================================#

		# Inicializacion variables
		self._counter = 0
		self._out = False



if __name__ == '__main__':

	try:
		# start the node
		rospy.init_node(skill_name)

		# create and spin the node
		node = ClientServerTestSkill()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass

