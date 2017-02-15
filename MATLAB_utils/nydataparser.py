"""
File: nydataparser

This file contains the functionality necessary to take a CSV file containing
NYC taxi data and turn it into readable data for the MATLAB algorithms to 
run through.

It also contains the functionality to turn a month of taxi data into one day.

Usage:

python3 nydataparser.py /path/to/CSV/file.csv
"""

import sys
from collections import namedtuple
import numpy as np
import scipy.io as sio
import os.path
import math
import datetime
import planparser
from datetime import datetime as dt
import csv
import matlab.engine
import matlab

NUM_PASSENGERS = 250
START_HOUR = 0
END_HOUR = 24
WEST_BOUNDARY = -74.0218
EAST_BOUNDARY = -73.9208
NORTH_BOUNDARY = 40.8870
SOUTH_BOUNDARY = 40.6968
DAYS = [4, 5, 6, 7, 8]

trip = namedtuple('trip', 'starttime pickupcoord dropoffcoord')
adjustedtrip = namedtuple('adjustedtrip', 'start finish time')

"""if  checkValidTrip(starttime.hour, pickupcoord, dropoffcoord, polyx, polyy, eng):
						currtrip = createMATTrip(trip(starttime, pickupcoord, dropoffcoord), locations)
						trips.append(currtrip)
						count += 1
						print(len(trips))
						with open('res/NYCtaxitrips.csv', 'a') as f:
							tripwriter = csv.writer(f)
							tripwriter.writerow([currtrip.start, currtrip.finish, currtrip.time])
					"""
"""
Function writeSingleDay

This function takes in the name of a New York Taxi CSV file and 
writes one single day to a csv file.
"""
def writeSingleDay(filename, RefLocation):
	print("Loading trips...")
	eng = matlab.engine.start_matlab() #for the lla2flat function
	trips = []
	with open(filename, 'r', 1) as f:
		reader = csv.reader(f)
		reader.__next__()
		reader.__next__()
		count = 0
		for item in reader:
			starttime = dt.strptime(item[1], '%Y-%m-%d %H:%M:%S')
			endtime = dt.strptime(item[2], '%Y-%m-%d %H:%M:%S')
			try:
				pickuplatlong = (float(item[6]), float(item[5]))
				dropofflatlong = (float(item[10]), float(item[9]))
				if starttime.day == 1:		
					pickupcoord = convertCoord(pickuplatlong, RefLocation, eng)
					dropoffcoord = convertCoord(dropofflatlong, RefLocation, eng)
					triptime = calculateTripTime(starttime, endtime)
					with open('res/November1data.csv', 'a') as f:
						writer = csv.writer(f)
						writer.writerow([11, starttime.day, starttime.hour, starttime.minute, starttime.second,\
										 pickupcoord[0], pickupcoord[1], dropoffcoord[0], dropoffcoord[1],\
										 pickuplatlong[0], pickuplatlong[1], dropofflatlong[0], dropofflatlong[1], \
										 triptime, float(item[4])])
						print(count)
						count += 1
					
			except ValueError:
				continue
				#if count >= NUM_PASSENGERS:
					#break
	#return trips

def calculateTripTime(starttime, endtime):
	t1 = starttime
	t2 = endtime
	return (t2 - t1).total_seconds()

"""
Funciton prelimInNYC

This function takes in a coordinate and does a preliminary check
to see whether or not the coordinate lies in a box specified by
the four boundary coordinates. This is a rough check to see if 
the coordinate is in NYC.
"""
def prelimInNYC(coord):
	latgood = SOUTH_BOUNDARY < coord[0] < NORTH_BOUNDARY
	longood = WEST_BOUNDARY < coord[1] < EAST_BOUNDARY
	return latgood and longood

"""
Function checkValidTrip

This function takes in a startime, pickup and dropoff locations, a 
polygon describing Manhattan, and a MATLAB engine object and returns
whether or not the trip is valid.
"""
def checkValidTrip(starthour, pickup, dropoff, polyx, polyy, eng):
	#print("Checking validity")
	valid = True
	valid = START_HOUR <= starthour <= END_HOUR \
			and inNYC(pickup, polyx, polyy, eng) \
			and inNYC(dropoff, polyx, polyy, eng)
	return valid

"""
Function inNYC

This function returns whether or not the given coordinate lies
in the polygon using MATLAB's inpolygon function
"""
def inNYC(coord, polyx, polyy, eng):
	return eng.inpolygon(coord[0], coord[1], polyx, polyy)

"""
Function convertCoord

This function takes in a latitude and longitude and returns the 
converted coordinate using MATLAB's lla2flat function.
"""
def convertCoord(coord, refloc, eng):
	coordtoconvert = matlab.double([coord[0],coord[1],0])
	refloc = matlab.double([refloc[0][0], refloc[0][1]])
	newcoord = eng.lla2flat(coordtoconvert, refloc, 28.8, 0)
	return (newcoord[0][0], newcoord[0][1])

"""
Function makeList

This function takes in an adjusted trip and turns it into a
list so that MATLAB will be able to parse it later.
"""
def makeList(trip):
	result = []
	for element in trip:
		result.append(element)
	return result

"""
Function createMATTrip

This function takes in a raw trip tuple and spits out an
adjustedtrip tuple that only contains the start node, end node,
and time.
"""
def createMATTrip(trip, locations):
	time = str(trip.starttime.time())
	startnode = planparser.findClosestNode(trip.pickupcoord[0], trip.pickupcoord[1], locations)
	endnode = planparser.findClosestNode(trip.dropoffcoord[0], trip.dropoffcoord[1], locations)

	return adjustedtrip(startnode, endnode, time)

"""
Function matify

This function takes in an array and prepares it to be a MATLAB
array so that it can be passed to MATLAB funcitons
"""	
def matify(arr):
	temparr = []
	for element in arr:
		temparr.append(element[0])
	length = len(temparr)
	arr = matlab.double(temparr)
	arr.reshape((length, 1))
	return arr

"""
Function parseCSV

This function takes in the name of a csv file representing taxi data
for New York City. It then writes to a .mat file a plans-like output
that contains sources, sinks, flows, and times.
"""
def parseCSV(filename):
	rawdata = planparser.loadMatData('bin/zhangNYdata.mat')
	locations = rawdata['NodesLocation']
	refloc = rawdata['RefLocation']
	polyx = rawdata['polyx']
	polyx = matify(rawdata['polyx'])
	polyy = matify(rawdata['polyy'])

	writeSingleDay(filename, refloc)
	#finaltrips = []
	#for trip in trips:
		#finaltrips.append(createMATTrip(trip, locations))

	#sources, sinks, times = planparser.createSourcesAndSinksAndTimes(finaltrips)
	#flowsIn = np.array([1 for x in range(len(finaltrips))])

	#outfile = 'bin/nyplans.mat'
	"""sio.savemat(outfile, {'Sources': sources[np.newaxis].T, 'Sinks': sinks[np.newaxis].T, \
						  'FlowsIn': flowsIn[np.newaxis].T, 'M' : len(sources), \
						  'Times': times[np.newaxis].T})
	"""

"""
Function turnMonthDataIntoDayData

This function takes in a month's worth of taxi data and 
writes one specific day to a file.
"""
def turnMonthDataIntoDayData(filename):
	with open(filename, 'r') as f:
		reader = csv.reader(f)
		reader.__next__()
		reader.__next__()
		for item in reader:
			starttime = dt.strptime(item[1], '%Y-%m-%d %H:%M:%S')
			if starttime.day in DAYS and START_HOUR < starttime.hour < END_HOUR:
				with open('res/Nov4-8data.csv', 'a') as outfile:
					print("wrote one!")
					writer = csv.writer(outfile)
					writer.writerow(item)
if __name__ == '__main__':
	parseCSV(sys.argv[1])