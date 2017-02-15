"""
File: pathwriter.py

This file contains the information necessary to turn routes from
the CDC16 passenger algorithms into readable XMl files for the
MATSim program.
"""

import xml.etree.ElementTree as ET
from xml.etree import ElementTree
from xml.dom import minidom
import sys
from collections import namedtuple
import numpy as np
import scipy.io as sio
import csv

plan = namedtuple('plan', 'person start finish route reb')
road = namedtuple('road', 'id start finish')

"""
Class: FileError

This class represents an exception when the given file 
is invalid. 
"""
class FileError(Exception):
	def __init__(self, filename, msg):
		self.filename = filename
		self.msg = msg

	def __str__(self):
		return repr(self.msg)

"""
Function loadPaths

This function takes in an array of paths from the CDC
algorithm and returns a list of routes represented by the 
nodes that the path goes through.
"""
def loadPaths(matpasspaths):
	cleanpaths = []
	for matpath in matpasspaths:
		cleanpath = []
		for element in matpath: #the data is reaaally nested in there
			for subelement in element:
				for nodes in subelement:
					for node in nodes:
						cleanpath.append(node[0])
		cleanpaths.append(cleanpath)
	return cleanpaths

"""
Function loadTimes

This function takes in an array of times from the CDC output
and turns it into a cleaner, more useful array of strings
that represent the times.
"""
def loadTimes(mattimes):
	cleantimes = []
	for mattime in mattimes:
		for time in mattime:
			cleantimes.append(time[0])

	return cleantimes

"""
Function loadRebPaths

This function takes in an output of the CDC algorithm for rebalancing
and returns a clean version of the paths as a list of links that need
to be traversed by each vehicle.
"""
def loadRebPaths(matrebpaths):
	cleanrebpaths = []
	for matrebpath in matrebpaths:
		for element in matrebpath:
			cleanrebpath = []
			for subelement in element:
				for nodes in subelement:
					for node in nodes:
						cleanrebpath.append(node[0])
			cleanrebpaths.append(cleanrebpath)

	return cleanrebpaths

"""
Function loadData

This function takes in a filename representing the output
of the CDC algorithms and returns a set of paths, times, and
a roadcapacity matrix that can be used by other member functions
of this file.
"""
def loadData(filename):
	matdata = sio.loadmat(filename)
	passpaths = matdata['passpaths']
	times = matdata['Times']
	origRoadCap = matdata['RoadCap']
	rebpaths = matdata['rebpaths']

	cleanpaths = loadPaths(passpaths)
	cleantimes = loadTimes(times)
	cleanrebpaths = loadRebPaths(rebpaths)
	
	return cleanpaths, cleantimes, origRoadCap, cleanrebpaths

"""
Function askForFileName

This function asks the user which file should be used for 
writing the XML data.
"""
def askForFileName():
	filename = input("Please enter an xml filename for modifying: ")
	while (filename[-4:] != '.xml'):
		filename = input("Invalid filename, try again: ")

	return 'bin/' + filename

"""
Function stringify

This function takes in a list and returns the list as a string
with each element separated by a space
"""
def stringify(route):
	result = ''
	for node in route:
		result += node + ' '

	return result[:-1]

"""
Function turnNodesIntoLinks

This function takes in a route, a set of roads, and the roadcapacity
matrix and returns the links needed to perform the route. It also
updates the capacity matrix so that it accurately reflects which 
cars are on the road.
"""
def turnNodesIntoLinks(route, roads, roadcap = None):
	links = []
	for i in range(len(route)-1):
		start = route[i]
		finish = route[i+1]
		for road in roads:
			if road.start == str(start) and road.finish == str(finish):
				links.append(road.id)
				if not roadcap == None:
					roadcap[start-1][finish-1] -= 1
	
	return links
	
"""
Function writeSinglePlan

This function takes in a plan (tree of XML file), a time, a route, 
a set of roads, and the roadcap matrix. It then writes an activity
from home to work with the route set by the CDC algorithm.
"""
def writeSinglePlan(plan, route = None, roads = None, roadcap = None, time = '12:00:00'):
	if len(route) == 0:
		return
	start = route[0]
	end = route[-1]

	listoflinks = turnNodesIntoLinks(route, roads, roadcap)

	act = ET.SubElement(plan, 'act')
	act.set('type', 'h')
	act.set('end_time', str(time))
	act.set('link', listoflinks[0])

	leg = ET.SubElement(plan, 'leg')
	leg.set('mode', 'car')

	route = ET.SubElement(leg, 'route')
	route.text = stringify(listoflinks)

	act2 = ET.SubElement(plan, 'act')
	act2.set('type', 'w')
	act2.set('link', listoflinks[-1])

"""
Function initXMLFile

This function takes in a set of routes, the road set
and the road capacity matrix and writes the entire
XML file by first asking for the filename, and going through
plan by plan and writing a single plan. It then cleans up
the XML and writes it to a file.
"""
def initXMLFile(routes, roads, roadcap):
	plans = ET.Element("plans")
	for i, item in enumerate(routes):
		time, route = item[0], item[1]
		person = ET.SubElement(plans, "person")
		person.set('id', str(i+1))
		plan = ET.SubElement(person, "plan")
		plan.set('selected', 'yes')
		writeSinglePlan(plan, route, roads, roadcap, time)
		
	return plans

"""
Function cleanXML

This function takes in the root of a raw XML tree and 
returns the XML as a clean, indented string.
"""
def cleanXML(root):
	rough = ElementTree.tostring(root, 'utf-8')
	reparsed = minidom.parseString(rough)

	return reparsed.toprettyxml(indent = '    ')

"""
Function loadRoads

This function takes in a filename representing a csv file
with all of the roads and loads the roads. A road is represented by a
tuple with an id, start node, and end node.
"""
def loadRoads(filename):
	roads = []
	with open(filename, 'r') as f:
		reader = csv.reader(f)
		for row in reader:
			roads.append(road(row[0], row[1], row[2]))

	return roads

"""
Function writeRebPaths

This function takes in rebalancing paths, plans (head of XML file),
a road network, and the number of passengers and writes the rest of
the XML file that was written by initXMLFile.

Returns the head of the XML file for writing purposes.
"""
def writeRebPaths(rebpaths, plans, roads, startind):
	for i, route in enumerate(rebpaths):
		person = ET.SubElement(plans, "person")
		person.set('id', str(i+1+startind))
		plan = ET.SubElement(person, "plan")
		plan.set('selected', 'yes')
		writeSinglePlan(plan, route, roads)

	return plans

"""
Function writePaths

This function takes in a filename representing the output of the
CDC algorithms and writes the XML file representing all of the routes
as each belonging to a single person.
"""
def writePaths(filename):
	#if filename[-4:] != ".mat":
	#	raise FileError(filename, "Please input a .mat file")

	#cleanpaths, cleantimes, origRoadCap, cleanrebpaths = loadData(filename)
	#routes = zip(cleantimes, cleanpaths)

	csvFileName = 'bin/roads.csv'
	roads = loadRoads(csvFileName)

	plans = initXMLFile(routes, roads, origRoadCap)

	#plans = writeRebPaths(cleanrebpaths, plans, roads, len(cleanpaths))

	parsed = cleanXML(plans)

	parsed = parsed[0:22] + '\n<!DOCTYPE plans SYSTEM "http://www.matsim.org/files/dtd/plans_v4.dtd">' + \
			 parsed[22:]

	outfilename = 'res/amodplans.xml'
	with open(outfilename, 'w') as f:
		f.write(parsed)

	#modifiedRoadCap = origRoadCap #orig was changed by initXMLFile, just changing var names

	#capfilename = 'res/modifiedRoadCap.mat'

	#sio.savemat(capfilename, {'RoadCap':modifiedRoadCap})

if __name__ == '__main__':
	writePaths(sys.argv[1])