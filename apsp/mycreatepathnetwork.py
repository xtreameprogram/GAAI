import sys, pygame, math, numpy, random, time, copy, operator
from pygame.locals import *

from constants import *
from utils import *
from core import *

# Creates a path node network that connects the midpoints of each nav mesh together
def myCreatePathNetwork(world, agent = None):
	nodes = []
	edges = []
	polys = []
	### YOUR CODE GOES BELOW HERE ###
	allnodes = []
	polys = computePolygons(world)
	# pathnodes at each poly center
	# centers = map(lambda t: ( sum(map(lambda p: p[0], t))/len(t), sum(map(lambda p: p[1], t))/len(t) ), polys)
	# Pathnodes on midpoint of each poly edge
	polynodes = [] # A polynode is a tuple, (polygon, list_of_pathnodes)
	for poly in polys:
		nodes = []
		last = None
		for point in poly:
			if last != None:
				#if distance(last, point) > world.agent.getRadius()/2:
				edgept = ((last[0]+point[0])/2.0, (last[1]+point[1])/2.0)
				tooclose = False
				for poly2 in polys:
					for p in poly2:
						if distance(edgept, p) < world.agent.rect.width:
							tooclose = True
				if not tooclose:
					nodes.append(edgept)
			last = point
		#if distance(poly[0], poly[len(poly)-1]) > world.agent.getRadius()/2:
		edgept = ((poly[0][0]+poly[len(poly)-1][0])/2.0, (poly[0][1]+poly[len(poly)-1][1])/2.0)
		tooclose = False
		for poly2 in polys:
			for p in poly2:
				if distance(edgept, p) < world.agent.rect.width:
					tooclose = True
		if not tooclose:
			nodes.append(edgept)
		polynodes.append((poly, nodes))
	for pn in polynodes:
		for p1 in pn[1]:
			for p2 in pn[1]:
				if p1 != p2:
					valid = True
					for o in world.obstacles:
						if pointOnPolygon(p1, o.getPoints()) or pointOnPolygon(p2, o.getPoints()) or pointOnPolygon(p1, [(0, 0), (SCREEN[0], 0), SCREEN, (0, SCREEN[1])]) or pointOnPolygon(p2, [(0, 0), (SCREEN[0], 0), SCREEN, (0, SCREEN[1])]):
							valid = False
					if valid:
						edge = (p1, p2)
						#if edge is already in the list, do not proceed
						same = False
						for e in edges:
							if (e[0] == edge[0] and e[1] == edge[1]) or (e[0] == edge[1] and e[1] == edge[0]):
								same = True
						if not same:
							# Make sure it is not too close to obstacles
							tooclose = False
							for p3 in world.getPoints():
								if minimumDistance(edge, p3) < world.agent.rect.width:
									tooclose = True
							if not tooclose:
#								pygame.draw.line(world.debug, (0, 0, 255), p1, p2, 1)
								edges.append(edge)
								allnodes.append(p1)
								allnodes.append(p2)
	allnodes = list(set(allnodes))
	nodes = allnodes
	### Try to take any dead-ends in edge network and connect them to somewhere else
	'''
	for point in nodes:
		if (map(lambda p:p[0], edges) + map(lambda p:p[1], edges)).count(point) == 1:
			# Point is a dead-end
			for point2 in nodes:
				if point != point2:
					hit = rayTraceWorld(point, point2, world.getLines())
					if hit == None:
						edge = (point, point2)
						tooclose = False
						for p in world.getPoints():
							if minimumDistance(edge, p) < world.agent.rect.width:
								tooclose = True
								break
						if not tooclose:
							edges.append((point, point2))
							# Ideally, get all possible connection points and use APSP to sort by path distance. This will connect up disjoint sub-graphs.
							break
	'''
	next, dist = APSPx(nodes, edges)
	for point in nodes:
		if (map(lambda p:p[0], edges) + map(lambda p:p[1], edges)).count(point) == 1:
			# Point is a dead-end
			sorteddist = sorted(dist.items(), key=operator.itemgetter(1))
			sorteddist.reverse()
			for x in sorteddist:
				hit = rayTraceWorld(point, x[0], world.getLines())
				if hit == None:
					edge = (point, x[0])
					tooclose = False
					for p in world.getPoints():
						if minimumDistance(edge, p) < world.agent.rect.width:
							tooclose = True
							break
					if not tooclose:
						edges.append(edge)
						break
	### YOUR CODE GOES ABOVE HERE ###
	return nodes, edges, polys

	
def computePolygons(world):
	triangles = []
	allpoints = world.getPoints()
	alllines = world.getLines()
	corerandom.shuffle(allpoints)
	for p1 in allpoints:
		for	p2 in allpoints:
			if p1 != p2:
				hitpoint = rayTraceWorldNoEndPoints(p1, p2, alllines)
				if hitpoint == None:
					appendLineNoDuplicates((p1, p2), alllines)

	# Make triangles
	for p1 in allpoints:
		successors = successorPoints(p1, alllines)
		for p2 in successors:
			successors2 = successorPoints(p2, alllines)
			for p3 in successors2:
				successors3 = successorPoints(p3, alllines)
				for p4 in successors3:
					if p4 == p1:
						triangles.append((p1, p2, p3))
	#remove duplicates
	triangles = map(lambda t: tuple(sorted(t)), triangles)
	triangles = list(set(triangles))
	#remove triangles inside of obstacles
	triangles2 = []
	for t in triangles:
		#if all triangle points are obstacle points, then remove if the center point is inside the obstacle
		ok = True
		for o in world.obstacles:
			if ok == True:
				if o.isInPoints(t[0]) and o.isInPoints(t[1]) and o.isInPoints(t[2]):
					if pointInsidePolygonPoints(( sum(map(lambda p:p[0], t))/3.0, sum(map(lambda p:p[1], t))/3.0 ), o.getPoints()):
						ok = False
#				elif o.isInPoints(t[0]) and o.isInPoints(t[1]) and o.twoAdjacentPoints(t[0], t[1]) == False:
#					ok = False
#				elif o.isInPoints(t[1]) and o.isInPoints(t[2]) and o.twoAdjacentPoints(t[1], t[2]) == False:
#					ok = False
#				elif o.isInPoints(t[0]) and o.isInPoints(t[2]) and o.twoAdjacentPoints(t[0], t[2]) == False:
#					ok = False
		if ok == True:
			triangles2.append(t)
	triangles3 = []
	# if there is someone inside of me, remove me
	for t1 in triangles2:
		ok = True
		for t2 in triangles2:
			if t1 != t2 and ok == True:
				different = list(set(t2) - set(t1))
				if len(different) == 1:
					d = different[0]
					if pointInsidePolygonPoints(d, t1):
						ok = False
		if ok:
			triangles3.append(t1)
	#assert: no polygons are overlapping
#	for t in triangles3:
#		drawCross(world.debug, ( sum(map(lambda p:p[0], t))/len(t), sum(map(lambda p:p[1], t))/len(t) ) )
	#reduce the number of triangles by merging into convex polygons
	polygons = triangles3
#	for x in polygons:
#		drawPolygon(x, world.debug, (255, 0, 0), 3, True)
	newpolygons = []
	oldpolygons = []
	for poly1 in polygons:
		for poly2 in polygons:
			if poly1 != poly2 and poly2 not in oldpolygons and poly1 not in oldpolygons:
				if polygonsAdjacent(poly1, poly2):
					poly3 = tuple(set(list(poly1) + list(poly2)))
					if isConvex(poly3):
						newpolygons.append(poly3)
						oldpolygons.append(poly1)
						oldpolygons.append(poly2)
						break
	polygons = newpolygons + list(set(polygons) - set(oldpolygons))
	return polygons

				
								
	






	

		
def polygonsOverlap(poly1, poly2):
	overlaps = []
	#Lines of poly1
	lines1 = []
	last = None
	for p in poly1:
		if last != None:
			lines1.append((last, p))
		last = p
	lines1.append((poly1[0], poly1[len(poly1)-1]))
	#lines of poly2
	lines2 = []
	last = None
	for p in poly2:
		if last != None:
			lines2.append((last, p))
		last = p
	lines2.append((poly2[0], poly2[len(poly2)-1]))
	#Check to see if any lines from poly1 are in poly2
	for l in lines1:
		center = ( sum(map(lambda p: p[0], l))/2.0, sum(map(lambda p: p[1], l))/2.0 )
		if pointInsidePolygon(center, lines2):
			overlaps.append(l[0])
			overlaps.append(l[1])
	if len(overlaps) > 0:
		return overlaps
	else:
		return False
	
	

#Return the the points that are link to the given point by some line	
def successorPoints(point, lines):
	result = set()
	for l in lines:
		if (l[0] == point):
			result.add(l[1])
		elif(l[1] == point):
			result.add(l[0])
	return list(result)
	

	


def APSPx(nodes, edges):
	dist = {} # a dictionary of dictionaries. dist[p1][p2] will give you the distance.
	next = {} # a dictionary of dictionaries. next[p1][p2] will give you the next node to go to, or None
	for n in nodes:
		next[n] = {}
		dist[n] = {}
	### YOUR CODE GOES BELOW HERE ###
	for n1 in nodes:
		for n2 in nodes:
			next[n1][n2] = None
			next[n2][n1] = None
			if n1 == n2:
				dist[n1][n2] = 0
				dist[n2][n1] = 0
			else:
				dist[n1][n2] = INFINITY
				dist[n2][n1] = INFINITY
	d = INFINITY
	for e in edges:
		d = distance(e[0], e[1])
		dist[e[0]][e[1]] = d
		dist[e[1]][e[0]] = d
	for k in nodes:
		for i in nodes:
			for j in nodes:
				if dist[i][k] + dist[k][j] < dist[i][j]:
					dist[i][j] = dist[i][k] + dist[k][j]
					next[i][j] = k
	### YOUR CODE GOES ABOVE HERE ###
	return next, dist

