'''
 * Copyright (c) 2014, 2015 Entertainment Intelligence Lab, Georgia Institute of Technology.
 * Originally developed by Mark Riedl.
 * Last edited by Mark Riedl 05/2015
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
'''

import sys, pygame, math, numpy, random, time, copy
from pygame.locals import * 

from constants import *
from utils import *
from core import *
from mycreatepathnetwork import *
from mynavigatorhelpers import *


###############################
### AStarNavigator
###
### Creates a path node network and implements the A* algorithm to create a path to the given destination.
			
class AStarNavigator(NavMeshNavigator):

	def __init__(self):
		NavMeshNavigator.__init__(self)
		

	### Create the path node network.
	### self: the navigator object
	### world: the world object
	def createPathNetwork(self, world):
		self.pathnodes, self.pathnetwork, self.navmesh = myCreatePathNetwork(world, self.agent)
		return None
		
	### Finds the shortest path from the source to the destination using A*.
	### self: the navigator object
	### source: the place the agent is starting from (i.e., its current location)
	### dest: the place the agent is told to go to
	def computePath(self, source, dest):
		self.setPath(None)
		### Make sure the next and dist matrices exist
		if self.agent != None and self.world != None: 
			self.source = source
			self.destination = dest
			### Step 1: If the agent has a clear path from the source to dest, then go straight there.
			###   Determine if there are no obstacles between source and destination (hint: cast rays against world.getLines(), check for clearance).
			###   Tell the agent to move to dest
			### Step 2: If there is an obstacle, create the path that will move around the obstacles.
			###   Find the path nodes closest to source and destination.
			###   Create the path by traversing the self.next matrix until the path node closest to the destination is reached
			###   Store the path by calling self.setPath()
			###   Tell the agent to move to the first node in the path (and pop the first node off the path)
			if clearShot(source, dest, self.world.getLinesWithoutBorders(), self.world.getPoints(), self.agent):
				self.agent.moveToTarget(dest)
			else:
				start = findClosestUnobstructed(source, self.pathnodes, self.world.getLinesWithoutBorders())
				end = findClosestUnobstructed(dest, self.pathnodes, self.world.getLinesWithoutBorders())
				if start != None and end != None:
					# print len(self.pathnetwork)
					newnetwork = unobstructedNetwork(self.pathnetwork, self.world.getGates())
					# print len(newnetwork)
					closedlist = []
					path, closedlist = astar(start, end, newnetwork)
					if path is not None and len(path) > 0:
						path = shortcutPath(source, dest, path, self.world, self.agent)
						self.setPath(path)
						if self.path is not None and len(self.path) > 0:
							first = self.path.pop(0)
							self.agent.moveToTarget(first)
		return None
		
	### Called when the agent gets to a node in the path.
	### self: the navigator object
	def checkpoint(self):
		myCheckpoint(self)
		return None

	### This function gets called by the agent to figure out if some shortcuts can be taken when traversing the path.
	### This function should update the path and return True if the path was updated.
	def smooth(self):
		return mySmooth(self)

	def update(self, delta):
		myUpdate(self, delta)


def unobstructedNetwork(network, worldLines):
	newnetwork = []
	for l in network:
		hit = rayTraceWorld(l[0], l[1], worldLines)
		if hit == None:
			newnetwork.append(l)
	return newnetwork

def get_f_value(initial, goal, new): #Gets the f, g, and h values for a node
	g = distance(initial, new)
	h = distance(new, goal)
	return g + h, g, h

def get_lowest_f(initial, goal, list):
	current_lowest = (list[0], get_f_value(initial, goal, list[0])[0])#Just iterates through all of the nodes and grabs the one with the lowest F value
	for node in list:
		if get_f_value(initial, goal, node)[0] < current_lowest[1]:
			current_lowest = (node, get_f_value(initial, goal, node)[0])
	return current_lowest[0]


def astar(init, goal, network):
	#print network
	path = []
	open = []
	closed = []
	### YOUR CODE GOES BELOW HERE ###
	neighbors = {} #Creates a dictionary to easily grab and store neighbor information
	for edge in network: 
		if edge[0] in neighbors:
			neighbors[edge[0]].append(edge[1])
		else:
			neighbors[edge[0]] = []
			neighbors[edge[0]].append(edge[1])

		if edge[1] in neighbors:
			neighbors[edge[1]].append(edge[0])
		else:
			neighbors[edge[1]] = []
			neighbors[edge[1]].append(edge[0])

	# print neighbors
	open.append(init)
	while open:
		current_node = get_lowest_f(init, goal, open)
		if current_node == goal:
			path.append(goal)
			return path, closed
		open.remove(current_node)
		closed.append(current_node)
		for neighbor in neighbors[current_node]:
			if neighbor not in closed:
				if neighbor not in open:
					open.append(neighbor)
		path.append(current_node)
		
	### YOUR CODE GOES ABOVE HERE ###
	return [], closed


def myUpdate(nav, delta):
	### YOUR CODE GOES BELOW HERE ###
	
	### YOUR CODE GOES ABOVE HERE ###
	return None



def myCheckpoint(nav):
	### YOUR CODE GOES BELOW HERE ###

	### YOUR CODE GOES ABOVE HERE ###
	return None


### Returns true if the agent can get from p1 to p2 directly without running into an obstacle.
### p1: the current location of the agent
### p2: the destination of the agent
### worldLines: all the lines in the world
### agent: the Agent object
def clearShot(p1, p2, worldLines, worldPoints, agent):
	### YOUR CODE GOES BELOW HERE ###
	agentL = 25
	for line in worldLines:
		rt1 = rayTrace(p1, p2, line)
		rt2 = rayTrace((p1[0]-agentL, p1[1]), (p2[0]-agentL, p2[1]), line)
		rt3 = rayTrace((p1[0]+agentL, p1[1]), (p2[0]+agentL, p2[1]), line)
		rt4 = rayTrace((p1[0], p1[1]-agentL), (p2[0], p2[1]-agentL), line)
		rt5 = rayTrace((p1[0], p1[1]+agentL), (p2[0], p2[1]+agentL), line)
		if rt1 is not None or rt2 is not None or rt3 is not None or rt4 is not None or rt5 is not None:
			return False
	### YOUR CODE GOES ABOVE HERE ###
	return True

