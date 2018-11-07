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
skill_name = 'client_model_skill'

server_name = 'server_model_skill'


class Client():
	
	def __init__(self):
		"""
		Init method.
		"""
		
		# class variables
		print("Init")
	
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


if __name__ == '__main__':

	try:
		# start the node
		rospy.init_node(pkg_name)
		
		print("Creo el cliente")
		cliente = Client()
		while(True):
			answer = raw_input("Inserta string: 1-> Activar cliente, 0-> Exit\n")
			if answer == "0":
				raise rospy.ROSInterruptException
				
			text = raw_input("Escribe el goal: ")
			resultado = cliente.client(text)
			print("Resultado: " + str(resultado))

	except rospy.ROSInterruptException:
		pass
