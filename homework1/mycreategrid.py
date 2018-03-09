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

# Creates a grid as a 2D array of True/False values (True =  traversable). Also returns the dimensions of the grid as a (columns, rows) list.
def myCreateGrid(world, cellsize):
	worldSize = world.getDimensions()
	dimensions = (int(worldSize[0]/cellsize), int(worldSize[1]/cellsize))

	# print worldSize, dimensions
	r = time.time()
	rayTraceWorld((0,0), (10, 9), world.getLines())
	print time.time() - r

	grid = [[True for x in xrange(0, worldSize[0] + 1)] for y in xrange(0, worldSize[1] + 1)]
	
	for i in xrange(0, dimensions[0] + 1):
		for j in xrange(0, dimensions[1] + 1):
			x = i * cellsize
			y = j * cellsize
			# if obj.pointInside((x, y)) or obj.pointInside((x + cellsize, y)) or obj.pointInside((x, y + cellsize)) or obj.pointInside((x + cellsize, y + cellsize)):
			if rayTraceWorld((x, y), (x + cellsize, y), world.getLines()) is not None or rayTraceWorld((x, y), (x, y + cellsize), world.getLines()) is not None or rayTraceWorld((x, y + cellsize), (x + cellsize, y + cellsize), world.getLines()) is not None or rayTraceWorld((x + cellsize, y), (x + cellsize, y + cellsize), world.getLines()) is not None:
				grid[i][j] = False

	return grid, dimensions

