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

# Creates a pathnode network that connects the midpoints of each navmesh together
def myCreatePathNetwork(world, agent = None):
	nodes = []
	edges = []
	polys = []

	obstacleLines = world.getLines()[4:]
	worldLines = set(world.getLines())
	worldPoints = world.getPoints()
	worldObstacles = world.getObstacles()
	numPoints = len(worldPoints)
	lineDict = {}
	polySet = set()

	# Create triangles that don't intersect with each other or obstacles.
	for i in range(numPoints - 2):
		p1 = worldPoints[i]
		for j in range(i + 1, numPoints - 1):
			p2 = worldPoints[j]
			if rayTraceOther(world, p1, p2, worldLines) is None:
				for k in range(j + 1, numPoints):
					p3 = worldPoints[k]
					if rayTraceOther(world, p2, p3, worldLines) is None and rayTraceOther(world, p3, p1, worldLines) is None:
						triangle = (p1, p2, p3)
						obstacle = ManualObstacle(triangle)

						# Make sure no obstacles are completely inside the triangle.
						valid = True
						if valid:
							for o in worldObstacles:
								obstaclePoints = o.getPoints()
								if len(obstaclePoints) == 3:
									centroid = getCentroid(o.getPoints())
									if pointInsidePolygonPoints(centroid, triangle):
										valid = False
										break

						if valid:
							polySet.add(obstacle)
							# Register the triangle's lines for later use (when merging).
							for line in [(p1, p2), (p2, p3), (p3, p1)]:
								isObstacleLine = False
								lineSet = set(line)
								for obstacleLine in obstacleLines:
									if set(obstacleLine) == lineSet:
										isObstacleLine = True
										break
								if not isObstacleLine:
									if line in lineDict:
										lineDict[line].append(obstacle)
									else:
										reverse = (line[1], line[0])
										if reverse in lineDict:
											lineDict[reverse].append(obstacle)
										else:
											lineDict[line] = [obstacle]
								worldLines.add(line)

	# Merge triangles in a way that preserves convexity.
	removeLines = []
	for line in lineDict:
		if len(lineDict[line]) > 1:
			polygon1 = lineDict[line][0]
			polygon2 = lineDict[line][1]
			polygon1Points = list(polygon1.getPoints())
			polygon2Points = list(polygon2.getPoints())
			polygon1Lines = polygon1.getLines()
			polygon2Lines = polygon2.getLines()
			polygon1Length = len(polygon1Points)
			polygon2Length = len(polygon2Points)
			# Reverse the point order of a polygon if they are in the same direction
			polygon1Start, polygon1Forward = getPolygonOrder(line, polygon1)
			polygon2Start, polygon2Forward = getPolygonOrder(line, polygon2)
			if polygon1Forward == polygon2Forward:
				polygon2Points.reverse()
				polygon2Start = polygon2Length - polygon2Start - 2
				if polygon2Start == -1:
					polygon2Start = polygon2Length - 1

			# Combine the polygons and test for convexity.
			newPolygonPoints = None
			if polygon1Start == polygon1Length - 1:
				newPolygonPoints = copy.copy(polygon1Points)
			else:
				newPolygonPoints = polygon1Points[polygon1Start + 1:] + polygon1Points[:polygon1Start + 1]
			if polygon2Start == polygon2Length - 1:
				newPolygonPoints += polygon2Points[1:polygon2Length - 1]
			else:
				newPolygonPoints += polygon2Points[polygon2Start + 2:] + polygon2Points[:polygon2Start]

			if isConvex(newPolygonPoints):
				newPolygon = ManualObstacle(newPolygonPoints)
				convertLines(lineDict, polygon1, polygon1Lines, newPolygon)
				convertLines(lineDict, polygon2, polygon2Lines, newPolygon)

				polySet.remove(polygon1)
				polySet.remove(polygon2)
				polySet.add(newPolygon)
				removeLines.append(line)

	for line in removeLines:
		lineDict.pop(line)

	# drawCentroids(world, polySet)

	# Create the final nav mesh and create cliques out of the boundaries of each polygon.
	nodeSet = set()
	allEdges = []
	for polygon in polySet:
		polygonPoints = polygon.getPoints()
		polys.append(polygonPoints)
		lines = polygon.getLines()
		# Use the centers of the boundaries and the centroid of the polygon as nodes.
		validPoints = [getMidpointLine(line) for line in lines if line in lineDict or (line[1], line[0]) in lineDict]
		centroid = getCentroid(polygonPoints)
		validLength = len(validPoints)
		polygonLength = len(polygonPoints)
		for i in range(validLength):
			path = (validPoints[i], centroid)
			if polygonLength > 3 and checkClearPath(path, world, world.agent):
				# Route borders through the centroid in higher order polygons.
				allEdges.append(path)
				nodeSet.add(validPoints[i])
				nodeSet.add(centroid)
			else:
				# Link borders directly in triangles or if the centroid is unusable.
				for j in range(validLength):
					if i != j:
						path = (validPoints[i], validPoints[j])
						if checkClearPath(path, world, world.agent):
							allEdges.append(path)
							nodeSet.add(validPoints[i])
							nodeSet.add(validPoints[j])

	# Only use nodes that the agent can reach on the path network.
	# Use a BFS of sorts to get a connected graph.
	start = findClosestUnobstructed(world.agent.position, nodeSet, world.getLines())
	currentNodes = set([start])
	nodeSet.clear()
	while len(currentNodes) > 0:
		nextNodes = set()
		for node in currentNodes:
			removeEdges = []
			for edge in allEdges:
				used = False
				if edge[0] == node:
					nextNodes.add(edge[1])
					used = True
				elif edge[1] == node:
					nextNodes.add(edge[0])
					used = True
				if used:
					edges.append(edge)
					removeEdges.append(edge)
			for edge in removeEdges:
				allEdges.remove(edge)
		nodeSet.update(currentNodes)
		currentNodes = nextNodes

	nodes = list(nodeSet)

	return nodes, edges, polys

# Draws the locations of centroids for all passed-in obstacles.
def drawCentroids(world, polys):
	for poly in polys:
		centroid = getCentroid(poly.getPoints())
		drawCross(world.debug, centroid, (255, 0, 0))

# Gets the centroid of a polygon defined by points.
def getCentroid(poly):
	totalX = totalY = 0
	polygonLength = len(poly)
	for point in poly:
		totalX += point[0]
		totalY += point[1]
	centroid = (totalX / polygonLength, totalY / polygonLength)
	return centroid

# Find the intersection between a line and the given lines, excluding lines that the points are on.
def rayTraceOther(world, p1, p2, lines):
	# Check if the line goes into an obstacle (further than just touching the border).
	for obstacle in world.getObstacles():
		obstaclePoints = obstacle.getPoints()
		numPoints = len(obstaclePoints)
		match = -1
		for i in range(numPoints):
			# Check if the two points are on the polygon and non-adjacent.
			if obstaclePoints[i] == p1 or obstaclePoints[i] == p2:
				if match == -1:
					match = i
				else:
					difference = i - match
					if difference > 1 and difference < numPoints - 1:
						midpoint = getMidpoint(p1, p2)
						if obstacle.pointInside(midpoint):
							return midpoint
						else:
							# Accounts for concave polygons.
							obstacleLines = obstacle.getLines()
							for line in obstacleLines:
								valid = True
								for linePoint in line:
									if linePoint == p1 or linePoint == p2:
										valid = False
										break
								if valid:
									intersect = rayTrace(p1, p2, line)
									if intersect != None:
										return intersect
	return rayTraceWorld(p1, p2, filterLines([p1, p2], lines))

# Returns the midpoint of a line.
def getMidpointLine(line):
	return getMidpoint(line[0], line[1])

# Returns the midpoint between two points.
def getMidpoint(p1, p2):
	return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)

# Returns a list of lines that do not intersect with the given points.
def filterLines(points, lines):
	otherLines = []
	for line in lines:
		for point in points:
			valid = True
			for linePoint in line:
				if point == linePoint:
					valid = False
					break
			if valid:
				intersect = rayTrace(point, (-10, -10), line)
				if intersect != None:
					for i in range(2):
						if (between(point[i], intersect[i], intersect[i])):
							valid = False
							break
			if not valid:
				break
		if valid:
			otherLines.append(line)
	return otherLines

# Checks if a polygon's points are in the same order as the given line.
# Also returns the index where the line is contained in the polygon.
def getPolygonOrder(line, polygon):
	polygonPoints = polygon.getPoints()
	polygonLength = len(polygonPoints)
	for i in range(polygonLength):
		if polygonPoints[i] == line[0]:
			if polygonPoints[i + 1] == line[1]:
				return i, True
			else:
				return polygonLength - 1, False
		elif polygonPoints[i] == line[1]:
			if polygonPoints[i + 1] == line[0]:
				return i, False
			else:
				return polygonLength - 1, True
	return -1, True

# Converts old polygons to new polygons in the line dictionary.
def convertLines(lineDict, polygon, polygonLines, newPolygon):
	for oldLine in polygonLines:
		if oldLine in lineDict:
			lineDict[oldLine] = [newPolygon if p == polygon else p for p in lineDict[oldLine]]
		else:
			reverse = (oldLine[1], oldLine[0])
			if reverse in lineDict:
				lineDict[reverse] = [newPolygon if p == polygon else p for p in lineDict[reverse]]

# Gets the extremes of a polygon.
def getBounds(polygon):
	minX = maxX = polygon[0][0]
	minY = maxY = polygon[0][1]
	polygonLength = len(polygon)
	for i in range(1, polygonLength):
		minX = min(minX, polygon[i][0])
		maxX = max(maxX, polygon[i][0])
		minY = min(minY, polygon[i][1])
		maxY = max(maxY, polygon[i][1])
	return minX, minY, maxX, maxY

# Checks if the agent can fit in a path.
def checkClearPath(line, world, agent):
	radius = agent.getMaxRadius()
	# Find the bounding box of the path to check for collisions with obstacles.
	node1 = numpy.matrix(line[0])
	node2 = numpy.matrix(line[1])
	between = node2 - node1
	backOffset = normalize(between) * radius
	sideOffset = backOffset * numpy.matrix('0,1;-1,0')
	boundingBox = [vecToList(node1 - backOffset + sideOffset),
					vecToList(node1 - backOffset - sideOffset),
					vecToList(node2 + backOffset - sideOffset),
					vecToList(node2 + backOffset + sideOffset)]

	# Check to see if any obstacles intersect the path.
	valid = True
	for obstacle in world.obstacles:
		for line in obstacle.getLines():
			if rayTrace(boundingBox[0], boundingBox[3], line) != None or rayTrace(boundingBox[1], boundingBox[2], line) != None:
				valid = False
				break
		# Edge case of a small obstacle entirely within the sides of the path.
		if valid and pointInsidePolygonPoints(obstacle.getPoints()[0], boundingBox):
			valid = False
		if not valid:
			break
	return valid

# Normalizes a vector.
def normalize(vector):
	norm = numpy.linalg.norm(vector)
	if norm == 0:
		return vector
	else:
		return vector / norm

# Converts a numpy matrix vector to a list.
def vecToList(vector):
	return vector.tolist()[0]