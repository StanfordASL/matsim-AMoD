"""
File: planparser.py

This file contains the functionality to parse a MATSim plans/population
file into the data needed by the MATLAB algorithms. It also contains
auxilary functionality that makes the calculation of this data easier.

Usage:

python3 planparser.py /path/to/plans/file.xml
"""

import xml.etree.ElementTree as ET
import sys
from collections import namedtuple
import numpy as np
import scipy.io as sio
import os.path
import math
import datetime
from datetime import datetime as dt

trip = namedtuple("trip", "start finish time")
Time = namedtuple("Time", "hours minutes seconds")

"""
Function: loadMatData
This function reads a network .mat file and extracts
the rawdata from it for use in the rest of the program.
"""
def loadMatData(filename):
	rawdata = sio.loadmat(filename)
	return rawdata

"""
Function findDistance
This function finds the pythagorean distance between a Cartesian point.
And a location on the map. The location is stored as (lat, long), 
which corresponds to (y,x)
"""
def findDistance(x,y, location):
	locX = location[0]
	locY = location[1]

	return math.sqrt((float(locX)-float(x))**2 + (float(locY)-float(y))**2)

"""
Function findClosestNode
This function takes in a position and a list of locations and finds the closest
node to a given (x,y) coordinate.
"""
def findClosestNode(x, y, locations): #locations should be in lat/long format, not x,y
	closest = -1
	shortestdistance = -1
	for i, location in enumerate(locations):
		distance = findDistance(x, y, location)
		if shortestdistance < 0 or distance < shortestdistance:
			closest = i+1
			shortestdistance = distance

	return closest

"""
Function calcTime
This function takes a piece of a plan and an old time and
determines what the end time of the activity is. If the time of 
activity is given as a duration, then that duration is added 
to the old time.
"""
def calcTime(piece, oldtime):
	if piece.attrib.get('end_time') != None:
		leavetime = piece.attrib.get('end_time')
		if len(leavetime) == 8:
			time = dt.strptime(leavetime, '%H:%M:%S').time()
			#time = Time(timeobj.hours, timeobj.minutes, timeobj.seconds)
		else:
			time = dt.strptime(leavetime, '%H:%M').time()
			#time = Time(timeobj.hours, timeobj.minutes, 0)
		return time

	elif piece.attrib.get('dur') != None:
		duration = piece.attrib.get('dur')
		if len(duration) == 8:
			temptime = dt.strptime(duration, '%H:%M:%S').time()
			deltaT = datetime.timedelta(hours=temptime.hour, minutes=temptime.minute, \
										seconds=temptime.second)
		else:
			temptime = dt.strptime(duration, '%H:%M')
			deltaT = datetime.timedelta(hours=temptime.hour, minutes=temptime.minute)
		return (datetime.datetime(100,1,1,oldtime.hour, oldtime.minute, oldtime.second) + deltaT).time()

"""
Function parsePlan
This function takes in a plan and a list of locations and
returns the trips that a person will take in a particular day.
"""
def parsePlan(plan, locations):
	trips = []
	triptime = datetime.time(0,0,0)
	lastplace = None
	for piece in plan:
		if piece.tag == 'act':
			currX = piece.attrib['x']
			currY = piece.attrib['y']
			closestNode = findClosestNode(currX, currY, locations)
			if lastplace != None:
				trips.append(trip(lastplace,closestNode, triptime))
			lastplace = closestNode
			triptime = calcTime(piece, triptime)
	return trips

"""
Function createSourcesAndSinks
This function takes in a list of trips and returns an mx1 array of 
sources and sinks such that the ith trip has origin sources[i] and 
destination sinks[i]
"""
def createSourcesAndSinksAndTimes(finaltrips):
	sources = np.zeros(len(finaltrips))
	sinks = np.zeros(len(finaltrips))
	times = np.zeros(len(finaltrips), dtype=np.object)
	for i, trip in enumerate(finaltrips):
		sources[i] = int(trip.start)
		sinks[i] =int(trip.finish)
		times[i] = str(trip.time)

	return sources,sinks, times

"""
Function parsePopulation
This function is the main method of this program and takes in an 
xml filename and writes the data obtained through other methods to a
.mat file that can later be used by the optimization algorithm.
""" 
def parsePopulation(filename):
	tree = ET.parse(filename)
	population = tree.getroot()

	finaltrips = []

	rawdata = loadMatData('bin/newyork.mat')
	locations = rawdata['Locations']

	for person in population:
		for plan in person: 
			# if there is only one plan choose it, otherwise choose the selected one
			if len(person) == 1 or plan.attrib['selected'] == 'yes':
				singletrips = parsePlan(plan, locations)
				for trip in singletrips:
					finaltrips.append(trip)

	sources, sinks, times = createSourcesAndSinksAndTimes(finaltrips)
	flowsIn = np.array([1 for x in range(len(finaltrips))])
	
	#outfile = "bin/" + filename[4:-4] + '.mat'
	outfile = 'test_plans.mat'
	sio.savemat(outfile, {'Sources': sources[np.newaxis].T, 'Sinks': sinks[np.newaxis].T, \
						  'FlowsIn': flowsIn[np.newaxis].T, 'M' : len(sources), \
						  'Times': times[np.newaxis].T})
 
if __name__ == "__main__":
	parsePopulation(sys.argv[1])