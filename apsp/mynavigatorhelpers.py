import sys, pygame, math, numpy, random, time, copy
from pygame.locals import * 

from constants import *
from utils import *
from core import *



### This function optimizes the given path and returns a new path
### source: the current position of the agent
### dest: the desired destination of the agent
### path: the path previously computed by the Floyd-Warshall algorithm
### world: pointer to the world
def shortcutPath(source, dest, path, world, agent):
	### YOUR CODE GOES BELOW HERE ###
	# alllines = world.getLines()
	# newstart = None
	# newend = None
	# for p in path:
	# 	fronthit = rayTraceWorld(source, p, alllines)
	# 	if fronthit == None:
	# 		tooclose = False
	# 		for p1 in world.getPoints():
	# 			if minimumDistance((source, p), p1) < world.agent.getRadius()*2.0:
	# 				tooclose = True
	# 		if not tooclose:
	# 			newstart = p
	# 	if newend == None:
	# 		backhit = rayTraceWorld(p, dest, alllines)
	# 		if backhit == None:
	# 			tooclose = False
	# 			for p1 in world.getPoints():
	# 				if minimumDistance((dest, p), p1) < world.agent.getRadius()*2.0:
	# 					tooclose = True
	# 			if not tooclose:
	# 				newend = p
	# newpath = []
	# start = False
	# end = False
	# for p in path:
	# 	if end == False:
	# 		if start == False:
	# 			if p == newstart:
	# 				newpath.append(p)
	# 				start = True
	# 		else:
	# 			newpath.append(p)
	# 		if p == newend:
	# 			newpath.append(p)
	# 			end = True
	# path = newpath
	### YOUR CODE GOES BELOW HERE ###
	return path

### Determine if there is a shortcut in the path network
### Return True if there is a shortcut or False if there is not.
### nav: the navigator object
def myCheckForShortcut(nav):
	# if nav.path != None and nav.agent.moveTarget != nav.destination:
	# 	hit = rayTraceWorld(nav.agent.rect.center, nav.destination, nav.world.getLines())
	# 	if hit == None:
	# 		tooclose = False
	# 		for p in nav.world.getPoints():
	# 			if minimumDistance((nav.agent.rect.center, nav.destination), p) < nav.agent.getRadius()*2.0:
	# 				tooclose = True
	# 		if not tooclose:
	# 			return True
	return False

### This function changes the move target of the agent if there is an opportunity to walk a shorter path.
### This function should call nav.agent.moveToTarget() if an opportunity exists and may also need to modify nav.path.
### nav: the navigator object
### This function returns True if the moveTarget and/or path is modified and False otherwise
def mySmooth(nav):
	### YOUR CODE GOES BELOW HERE ###
	# if nav.path != None and nav.agent.moveTarget != nav.destination:
	# 	if myCheckForShortcut(nav):
	# 		nav.path = []
	# 		nav.agent.moveToTarget(nav.destination)
	# 		return True
	# 	elif canSmooth(nav):
	# 		next = nav.path.pop(0)
	# 		nav.agent.moveToTarget(next)
	# 		return True
	### YOUR CODE GOES ABOVE HERE ###
	return False


def canSmooth(nav):
	# if nav.path != None and len(nav.path) > 0:
	# 	next = nav.path[0]
	# 	hit = rayTraceWorld(nav.agent.rect.center, next, nav.world.getLines())
	# 	if hit == None:
	# 		tooclose = False
	# 		for p in nav.world.getPoints():
	# 			if minimumDistance((nav.agent.rect.center, next), p) < nav.agent.getRadius()*2.0:
	# 				tooclose = True
	# 		if tooclose:
	# 			return False
	# 		else:
	# 			return True
	return False


