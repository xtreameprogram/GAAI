'''
 * Copyright (c) 2014, 20agentL Entertainment Intelligence Lab, Georgia Institute of Technology.
 * Originally developed by Mark Riedl.
 * Last edited by Mark Riedl 05/20agentL
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

agentL = 25

# Creates the pathnetwork as a list of lines between all pathnodes that are traversable by the agent.
def myBuildPathNetwork(pathnodes, world, agent = None):
	lines = []
	for node in pathnodes:
		for oNode in pathnodes:
			if node is not oNode:
				flag = True
				for line in world.getLinesWithoutBorders():
					rt1 = rayTrace(node, oNode, line)
					rt2 = rayTrace((node[0]-agentL, node[1]), (oNode[0]-agentL, oNode[1]), line)
					rt3 = rayTrace((node[0]+agentL, node[1]), (oNode[0]+agentL, oNode[1]), line)
					rt4 = rayTrace((node[0], node[1]-agentL), (oNode[0], oNode[1]-agentL), line)
					rt5 = rayTrace((node[0], node[1]+agentL), (oNode[0], oNode[1]+agentL), line)
					if rt1 is not None or rt2 is not None or rt3 is not None or rt4 is not None or rt5 is not None:
						flag = False
						break
				if flag:
					lines.append((node, oNode))
				

	return lines
