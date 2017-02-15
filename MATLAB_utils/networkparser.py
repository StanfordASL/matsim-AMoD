"""
File: networkparser.py

This file takes in an xml file that can be used by MATSim as a network
file and turns it into a dictionary that maps each node to all the neighbors
it can reach.

Usage:

python3 networkparser.py /path/to/network/file.xml
"""

import xml.etree.ElementTree as ET
import sys
from collections import namedtuple
import numpy as np
import scipy.io as sio
import csv
import math

#namedtuple containing link information
link = namedtuple("link", "id start finish capacity speed length")
road = namedtuple("road", "id start finish")
nodetuple = namedtuple("node", "id long lat")

"""
Function: parseNodes
This function takes in the nodes branch of the 
network file and turns it into a list of nodes
to be used as the keys for the dictionary.
"""
def parseNodes(branch):
	nodes = []
	for node in branch:
		nodes.append(nodetuple(node.attrib['id'], node.attrib['x'], node.attrib['y']))
	return nodes

"""
Function: parseLinks
This function takes in a dictionary called graph
which has the nodes as keys and maps all the nodes
to the nodes that can be reached from them.
It also keeps track of all the links and their necessary information.
***NB: nodes are given new ID numbers to make them all from 0...N-1****
instead of the random numbers that they could have been before.
"""
def parseLinks(links, branch, indexer):
	roads = []
	for piece in branch:
		#if lengths are really small, then it makes sense to magnify them by a factor of 1000
		#as it doesn't change the optimization problem
		if float(piece.attrib['length']) < .01 and float(piece.attrib['length']) > 0:
			length = str(float(piece.attrib['length'])*1000)
		else:
			length = piece.attrib['length']
		road = link(piece.attrib['id'], piece.attrib['from'], piece.attrib['to'], piece.attrib['capacity'],\
					piece.attrib['freespeed'], length)
		roads.append(road)
		links[indexer[road.start]].append(indexer[road.finish] + 1)

	return links, roads

"""
Function: createNodeIndexer
This function creates a dictionary that maps each node to an index
to be used in matrices.
"""
def createNodeIndexer(nodes):
	indexes = [x for x in range(len(nodes))] 
	return dict(((node.id,x) for x,node in zip(indexes,nodes)))

"""
Function: createTravelTimeMatrix
This function creates an nx3 matrix that has the following property:
the first column represent start nodes, the second column represents
end nodes, and the final column represents the travel time of the 
link represented by the first two columns..
"""
def createTravelTimeMatrix(nodes, links, indexer):
	matrix = np.zeros((len(links), 3))
	for i,link in enumerate(links):
		matrix[i][0] = int(indexer[link.start])
		matrix[i][1] = int(indexer[link.finish])
		matrix[i][2] = float(link.length)/float(link.speed)

	return matrix

"""
Function createCapacityMatrix

This function creates an nx3 matrix that has the following property:
the first column represent start nodes, the second column represents
end nodes, and the final column represents the capacity of the particular
link.
"""
def createCapacityMatrix(nodes, links, indexer):
	matrix = np.zeros((len(links), 3))
	for i,link in enumerate(links):
		matrix[i][0] = int(indexer[link.start])
		matrix[i][1] = int(indexer[link.finish])
		matrix[i][2] = link.capacity

	return matrix

"""
Function: createNodeLocationsMatrix
This function takes in a list of nodes and returns an Nx2 matrix
containing the latitudes and longitudes of each node. Note: latitude
comes first in the pairing even though it is the "y" coordinate in the
network.xml files
"""
def createNodeLocationsMatrix(nodes, rotangle):
	locations = np.zeros((len(nodes), 2))
	for i, node in enumerate(nodes):
		locations[i][0] = node.lat
		locations[i][1] = node.long
		if rotangle != 0:
			theta = math.radians(rotangle)
			rotmat = [[math.cos(theta), math.sin(theta)],\
					  [-math.sin(theta), math.cos(theta)]]
			locations[i] = np.dot(locations[i],rotmat)

	return locations

"""
Function: findReferenceLocation
This function finds the reference location that will be used as the origin
when MATLAB transforms the coordinates from latitude/longitude into Cartesian
coordiantes. The point is found by using a point a little bit to the bottom left
of the farthest southwest point.
"""
def findReferenceLocation(locations):
	minlat, maxlat = min(locations[:,0]), max(locations[:,0])
	minlong, maxlong = min(locations[:,1]), max(locations[:,1])
	
	reflatitude = minlat - 0.005*(maxlat - minlat)
	reflongitude = minlong - 0.005*(maxlong - minlong)

	return np.array([reflatitude, reflongitude])

"""
Function: parseNetwork
This function takes in the name of an xml file that will be converted
into a graph for MATLab to work with.
"""
def parseNetwork(filename):
	tree = ET.parse(filename)
	root = tree.getroot()

	for child in root:
		if child.tag == "nodes":
			print("Processing nodes...")
			nodes = parseNodes(child)
			indexer = createNodeIndexer(nodes)
			graph = dict((node.id,[]) for node in nodes)
		if child.tag == "links":
			print("Processing links...")
			links = np.zeros(len(nodes), dtype=object) #empty array which will eventually be converted to cell
			for i in range(len(links)): #initialize each entry to be an empty list
				links[i] = []
			RoadGraph, roads = parseLinks(links, child, indexer)
	
	print("Building traveltimes matrix...")
	traveltimes = createTravelTimeMatrix(nodes, roads, indexer)
	print("Building capacity matrix...")
	capacities = createCapacityMatrix(nodes, roads, indexer)
	print("Building locations matrix...")
	rotangle = 28.9
	locations = createNodeLocationsMatrix(nodes, rotangle)
	print("Finding reference location...")
	refLocation = findReferenceLocation(locations)

	print("Writing roads...")
	simpleroads = []
	for portion in roads:
		simpleroads.append(road(portion.id, portion.start, portion.finish))

	roadsOutFile = 'bin/roads.csv'
	with open(roadsOutFile, 'w') as f:
		w = csv.writer(f)
		w.writerows([(road.id, road.start, road.finish) for road in simpleroads])

	#filename = "bin/" + filename[4:-4] + ".mat" #custom filename
	filename = 'bin/newyork.mat'
	sio.savemat(filename, {'RoadGraph':RoadGraph, 'RoadCap': capacities, \
						   'TravelTimes':traveltimes, 'Locations': locations,\
						   'RefLocation': refLocation})
	
	

	print("Done.")


if __name__ == "__main__":
	parseNetwork(sys.argv[1])