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
### APSPNavigator
###
### Creates a path node network and implements the FloydWarshall all-pairs shortest-path algorithm to create a path to the given destination.
			
class APSPNavigator(NavMeshNavigator):

	### next: indicates which node to traverse to next to get to a given destination. A dictionary of dictionaries such that next[p1][p2] tells you where to go if you are at p1 and want to go to p2
	### dist: the distance matrix. A dictionary of dictionaries such that dist[p1][p2] tells you how far from p1 to p2.

	def __init__(self):
		NavMeshNavigator.__init__(self)
		self.next = None
		self.dist = None
		

	### Create the pathnode network and pre-compute all shortest paths along the network.
	### self: the navigator object
	### world: the world object
	def createPathNetwork(self, world):
		self.pathnodes, self.pathnetwork, self.navmesh = myCreatePathNetwork(world, self.agent)
		self.next, self.dist = APSP(self.pathnodes, self.pathnetwork)
		return None
		
	### Finds the shortest path from the source to the destination.
	### self: the navigator object
	### source: the place the agent is starting from (i.e., it's current location)
	### dest: the place the agent is told to go to
	def computePath(self, source, dest):
		### Make sure the next and dist matricies exist
		if self.agent != None and self.world != None and self.next != None and self.dist != None: 
			self.source = source
			self.destination = dest
			if clearShot(source, dest, self.world.getLinesWithoutBorders(), self.world.getPoints(), self.agent):
				self.agent.moveToTarget(dest)
			else:
				start = findClosestUnobstructed(source, self.pathnodes, self.world.getLines())
				end = findClosestUnobstructed(dest, self.pathnodes, self.world.getLines())
				print start, end
				if start != None and end != None and start in self.dist and end in self.dist[start] and self.dist[start][end] < INFINITY:
					path = findPath(start, end, self.next)
					path = shortcutPath(source, dest, path, self.world, self.agent)
					self.setPath(path)
					if len(self.path) > 0:
						first = self.path.pop(0)
						self.agent.moveToTarget(first)
		return None

	### This function gets called by the agent to figure out if some shortcutes can be taken when traversing the path.
	### This function should update the path and return True if the path was updated.
	def smooth(self):
		return mySmooth(self)










### Returns a path as a list of points in the form (x, y)
### start: the start node, one of the nodes in the path network
### end: the end node, one of the nodes in the path network
### next: the matrix of next nodes such that next[p1][p2] tells where to go next
def findPath(start, end, next):
	### YOUR CODE GOES BELOW HERE ###
	if next[start][end] is None:
		return []
	path = [start]
	while not (start is end):
		start = next[start][end]
		path.append(start)
	### YOUR CODE GOES ABOVE HERE ###

	return path



	
def APSP(nodes, edges):
	dist = {} # a dictionary of dictionaries. dist[p1][p2] will give you the distance.
	next = {} # a dictionary of dictionaries. next[p1][p2] will give you the next node to go to, or None
	for n in nodes:
		next[n] = {}
		dist[n] = {}
	### YOUR CODE GOES BELOW HERE ###
	for n in nodes:
		for n2 in nodes:
			dist[n][n2] = float("inf")
			next[n][n2] = None
	for edge in edges:
		point1 = edge[0]
		point2 = edge[1]
		dist[point1][point2] = distance(point1, point2)
		dist[point2][point1] = distance(point1, point2)
		next[point1][point2] = point2
		next[point2][point1] = point1
	for k in nodes:
		for i in nodes:
			for j in nodes:
				if dist[i][j] > dist[i][k] + dist[k][j]:
					dist[i][j] = dist[i][k] + dist[k][j]
					next[i][j] = next[i][k]
	# print next
	### YOUR CODE GOES ABOVE HERE ###
	return next, dist

### Returns true if the agent can get from p1 to p2 directly without running into an obstacle.
### p1: the current location of the agent
### p2: the destination of the agent
### worldLines: all the lines in the world
### agent: the Agent object
def clearShot(p1, p2, worldLines, worldPoints, agent):
	# return True
	### YOUR CODE GOES BELOW HERE ###
	# if rayTraceWorld(p1, p2, worldLines) is not None:
	# 	return False
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
