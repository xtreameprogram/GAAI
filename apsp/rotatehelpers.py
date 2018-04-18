import sys, pygame, math, numpy, random, time, copy
from pygame.locals import * 

from constants import *
from utils import *
from core import *
from apspnavigator import *
from nearestgatherer import *

# returns a point rotated by 90 degrees around center position
def RotatePoint(point, center):
	radians = math.pi/2

	p0 = point[0] - center[0]
	p1 = point[1] - center[1]

	px = p0 * math.cos(radians) - p1 * math.sin(radians)
	py = p0 * math.sin(radians) + p1 * math.cos(radians)

	p = (px + center[1], py + center[0])

	return p

def NewWorld(dimensions, terrainPoints, resourcePoints):
	newWorld = GameWorld(SEED, dimensions, dimensions)
	newWorld.initializeTerrain(terrainPoints)
	newWorld.initializeResources(resourcePoints, RESOURCE)
	newWorld.debugging = True
	return newWorld

# returns a world rotated by 90 degrees
def NewRotatedWorld(dimensions, terrainPoints, resourcePoints):
	center = (dimensions[0]/2, dimensions[1]/2)
	rotatedCenter = (center[1], center[0])

	rotatedDimensions = (dimensions[1], dimensions[0])

	rotatedTerrainPoints = RotatedTerrain(terrainPoints,center)

	rotatedResourcePoints = RotatedResources(resourcePoints,center)

	newWorld = NewWorld(rotatedDimensions, rotatedTerrainPoints, rotatedResourcePoints)
	return newWorld

def NewAgent(world, agentPosition):
	return Agent(AGENT, agentPosition, 0, SPEED, world)

# returns a new agent rotated by 90 degrees
def NewRotatedAgent(world, agentPosition):
	center = (world.getDimensions()[1]/2, world.getDimensions()[0]/2)
	rotatedAgentPosition = RotatePoint(agentPosition, center)
	agent = NewAgent(world, rotatedAgentPosition)
	return agent

def NewGatherer(world, gathererPosition):
	return NearestGatherer(NPC, gathererPosition, 0.0, SPEED, world)

# returns a new gatherer rotated by 90 degrees
def NewRotatedGatherer(world, gathererPosition):
	center = (world.getDimensions()[1]/2, world.getDimensions()[0]/2)
	rotatedGathererPosition = RotatePoint(gathererPosition, center)
	gatherer = NewGatherer(world, rotatedGathererPosition)
	return gatherer

# returns terrain points rotated by 90 degrees around center
def RotatedTerrain(terrainPoints, center):
	rotatedTerrainPoints = []
	for obstacle in terrainPoints:
		obstacle2 = []

		for p in obstacle:
			rotatedP = RotatePoint(p, center)
			obstacle2.append(rotatedP)

		rotatedTerrainPoints.append(obstacle2)

	return rotatedTerrainPoints

# returns resource positions rotated by 90 degrees around center
def RotatedResources(resourcePoints, center):
	rotatedResourcePoints = []
	for p in resourcePoints:
		rotatedP = RotatePoint(p, center)
		rotatedResourcePoints.append(rotatedP)

	return rotatedResourcePoints