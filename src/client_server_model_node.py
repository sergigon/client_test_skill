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

import roslib
import importlib
import rospy
import actionlib

# creates the ActionServer name pattern
import multimedia_msgs.msg
server_module_str = "multimedia_msgs.msg" # Libreria donde se encuentran los mensajes de los action
server_action_name = "Test" #### Nombre del mensaje .action que se quiere enviar ####
server_action_str = server_action_name + "Action"
server_feedback_str = server_action_name + "Feedback"
server_result_str = server_action_name + "Result"
# Usa este formato:
# my_import(server_module_str, server_feedback_str)
# Es igual a poner: multimedia_msgs.msg.{Action_name}Feedback

# Package name
pkg_name = 'client_test_skill'
roslib.load_manifest(pkg_name)

# declare this only if the name is different of 'pkg_name'
skill_name = "client_server_model_skill"

# ------------------------------------------- #
# Result:
# -1: Si ha habido algun error o cancelacion
# 0: Si todo va bien
# 1: Fail
# ------------------------------------------- #

# ------------------------------------------- #
# Feedback:
# Devuelve el numero del contador
# ------------------------------------------- #

class Client():
	
	def __init__(self):
		"""
		Init method.
		"""
		
		self.start_pub = rospy.Publisher(server_name+'/start', Empty, latch=True queue_size=1)
		self.stop_pub = rospy.Publisher(server_name+'/stop', Empty, latch=True, queue_size=1)
		
		# class variables
		print("Init")
	
	def skill_server_start(self, text=None):
		# Mas info en: https://asimov.uc3m.es/mini/state_machine/blob/indigo-devel/src/state_akinator.py
		# start the skill
		
		self.start_pub.publish(Empty())
		print server_name+"start"
		rospy.sleep(2)
		 
		result = self.client(text)
		 
		if result != None:
			if result.result == -1:
				aux_str = "ERROR"
			if result.result == 0:
				aux_str = "SUCCESS"
			if result.result == 1:
				aux_str = "FAIL"
			print "Result: ", aux_str
		
		print "proceso acabado"
			
		# stop the skill
		
		self.stop_pub.publish(Empty())
		print "Akinator_skill stop"
		rospy.sleep(1)
	
	def client(self, text=None):  
		print(skill_name + " creating client")
		
		# Creates the ActionClient
		clt = actionlib.ActionClient(server_name, multimedia_msgs.msg.TestAction)
		print "waiting for server " + server_name
		clt.wait_for_server()
		print "server ready"
		
		# Creates the goal
		if text==None:
			text = "hola te mando un goal desde el cliente " + skill_name
		goal = multimedia_msgs.msg.TestGoal(text)
		# Sends the goal to the action server
		print(pkg_name + " sending goal")
		clt_handle = clt.send_goal(goal)
		
		result = None
		
		if clt_handle != None:
			rospy.sleep(10.0)
			# wait for the server to finish performing the action
			cancel = False # Variable to cancel the goal
			once = False # Variable aux
			
			# Cojo el status del goal enviado, si esta en estado
			# PREEMPTED, SUCCEEDED o ABORTED, salgo de la ejecucion del cliente
			while clt_handle.get_goal_status() != 2 and clt_handle.get_goal_status()!=3 and clt_handle.get_goal_status() != 4:
				if cancel and not once:
					rospy.logwarn("Canceled!")
					once = True
					# cancel current goal
					clt_handle.cancel()
				
				################ Ejecucion del cliente #################
				# Introduce aqui lo que quieres que haga el cliente 
				# mientras espera el resultado.
				# Para cancelar el goal y salir del loop, iguala la
				# variable "cancel a True".
				
				
				# ....
				
				# ==================================================== #
				print skill_name + " waiting for finish"
				print("goal status: " + str(clt_handle.get_goal_status()))
				rospy.sleep(1.0)

			# sleep some time to wait the response
			rospy.sleep(1.0)
			result = clt_handle.get_result()
		return result


class ClientServerModelSkill(Skill):

	# Feedback and result of this skill
	_feedback = multimedia_msgs.msg.TestFeedback() # getattr(multimedia_msgs.msg, server_feedback_str) #my_import(server_module_str, server_feedback_str) # (multimedia_msgs.msg.{Action_name}Feedback)
	_result = multimedia_msgs.msg.TestResult() # getattr(multimedia_msgs.msg, server_result_str) # (multimedia_msgs.msg.{Action_name}Result)

	def __init__(self):
		"""
		Init method.
		"""
		# init the skill
		Skill.__init__(self, skill_name, CONDITIONAL)

		# class variables
		self._as = None # SimpleActionServer variable
		self._goal = 0 # Goal a recibir
		
		self._out = False # Variable de salida del loop en el execute_cb
		self._counter = 0 


	def create_msg_srv(self):
		"""
		This function has to be implemented in the children.
 		"""
 		print("create_msg_srv() called")
		# publishers and subscribers
		
		# servers and clients

		# Si el servidor actionlib no se ha inicializado:
		if not self._as:
			self._as = actionlib.SimpleActionServer(skill_name, multimedia_msgs.msg.TestAction, execute_cb=self.execute_cb, auto_start=False)

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

	def execute_cb(self, goal): # Se activa cuando recibes un goal
		"""
		Spinner of the node.
		"""
		rospy.loginfo('[' + pkg_name + ']')
	
		# default values (FAIL)
		self._result.result = 1
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
					if self._counter >= 10:
						self._result.result = 0
						self._out = True # Salgo del loop
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
		node = ClientServerModelSkill()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
