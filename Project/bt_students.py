#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# pickup cube
		b1 = pickup()
		# tuck the arm
		b2 = tuckarm()
		# lower head
		b3 = movehead_down()
		# move base
		b4 = movebase("pick")
		# localize
		b5 = localization()
		# upper head
		b6 = movehead_up()
		# respawn
		b7 = respawn()
		# move base
		b8 = movebase("place")
		# place cube
		b9 = place()
		# detect cube
		b10 = detect_aruco()
		# localized?
		b11 = localization_cond()
		# picked?
		b12 = pickup_cond()
		# placed?
		b13 = place_cond()
		# respwan
		b14 = respawn()
		

		# become the tree
		#tree = RSequence(name="Main sequence", children=[b2, b5, b4, b3, b1, b6, b8, b9])
		#tree = RSequence(name="Main sequence", children=[b6,b3,b6,b3])
		#tree = RSequence(name="Main sequence", children=[b6, b2, b5, b4, b3, b1, b6, b8, b9])
		#fallback = pt.composites.Selector(name = "fallback", children=[b8, b2])
		loc_fallback = pt.composites.Selector(name="loc fallback", children=[b11, b5])
		pick_sequence = pt.composites.Sequence(name="pick sequence", children=[b4, b3, b1, b6])
		picked_fallback = pt.composites.Selector(name="picked fallback", children=[b12, pick_sequence])
		detect_sequence = pt.composites.Sequence(name="detect sequence", children=[b3, b9, b10])
		respawn_fallback = pt.composites.Selector(name="respawn fallback", children=[detect_sequence, b14])
		place_sequence = pt.composites.Sequence(name="place sequence", children=[b8, respawn_fallback])
		placed_fallback = pt.composites.Selector(name="placed fallback", children=[b13, place_sequence])
		tree = RSequence(name="Main sequence", children=[b6, b2, loc_fallback, picked_fallback, placed_fallback])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
