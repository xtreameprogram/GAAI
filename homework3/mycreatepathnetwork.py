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

import sys, pygame, math, numpy, random, time, copy, operator
from pygame.locals import *

from constants import *
from utils import *
from core import *

def occurrence(point, lst):
	counter = 0
	for i in lst:
		if i == point:
			counter += 1
	return counter

def calcMid(p1, p2):
	return ((p1[0] + p2[0])/2., (p1[1] + p2[1])/2.)

def allConnect(oPoing, point, tem, worldLines):
	temp = list(tem)
	temp.append(oPoing)
	for node in temp:
		if node is not point:
			if rayTraceWorldNoEndPoints(point, node, worldLines) is not None or pointInsidePolygonLines(calcMid(point, node), worldLines):
				return False
	return True


# Creates a pathnode network that connects the midpoints of each navmesh together
def myCreatePathNetwork(world, agent = None):
	nodes = []
	edges = []
	polys = []

	agentL = 35
	lines = []

	dimensions = world.getDimensions()
	taboo = set([(0,0), (0, dimensions[1]), (dimensions[0], 0), (dimensions[0], dimensions[1])])
	# print(taboo)
	for point in world.getPoints():
		temp = set()

		for oPoint in  world.getPoints():
			# if oPoint == (0,0):
				# print point, oPoint, ((oPoint in taboo and point not in taboo) or (oPoint not in taboo and point in taboo) or (oPoint not in taboo and point not in taboo))
			if ((oPoint in taboo and point not in taboo) or (oPoint not in taboo and point in taboo) or (oPoint not in taboo and point not in taboo)) and rayTraceWorldNoEndPoints(point, oPoint, lines) is None:
				if allConnect(point, oPoint, temp, world.getLinesWithoutBorders()):
					temp.add(oPoint)
					temp.add(point)
					lines.append((point, oPoint))
		polys.append(list(temp))
	
	# print lines
	# for line in lines:
	# 	mid = calcMid(line[0], line[1])
	# 	drawCross(world.debug, mid, width = 3)
	# 	nodes.append(mid)

	# line = world.getLines()
	# used = []

	
	# for i in xrange(0, len(nodes)):
	# 	for j in xrange(0, len(nodes)):
	# 		rt1 = rayTraceWorld(nodes[i], nodes[j], line)
	# 		rt2 = rayTraceWorld((nodes[i][0]-agentL, nodes[i][1]), (nodes[j][0]-agentL, nodes[j][1]), line)
	# 		rt3 = rayTraceWorld((nodes[i][0]+agentL, nodes[i][1]), (nodes[j][0]+agentL, nodes[j][1]), line)
	# 		rt4 = rayTraceWorld((nodes[i][0], nodes[i][1]-agentL), (nodes[j][0], nodes[j][1]-agentL), line)
	# 		rt5 = rayTraceWorld((nodes[i][0], nodes[i][1]+agentL), (nodes[j][0], nodes[j][1]+agentL), line)
	# 		if rt1 is None and rt2 is None and rt3 is None and rt4 is None and rt5 is None and occurrence(nodes[i], used) < 3:
	# 			edges.append((nodes[i], nodes[j]))
	# 			used.append(nodes[i])
	# 			used.append(nodes[j])

	return nodes, list(set(edges)), polys

	
