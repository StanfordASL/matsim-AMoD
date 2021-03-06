import xml.etree.ElementTree as ET
from xml.etree import ElementTree
from xml.dom import minidom
import csv
import sys
import scipy.io as sio
import datetime
import pathwriter
import planparser
import random
import time as tm

TRAV_TIME_SECONDS = 60
START_HOUR = 0
END_HOUR = 24
count = 500

def turnCSVIntoPlans(filename, roadsIn, roadsOut, roadMap):
	plans = ET.Element("population")
	rawdata = planparser.loadMatData('bin/zhangNYDataOSM.mat')
	locations = rawdata['NodesLocation']

	with open(filename, 'r') as f:
		reader = csv.reader(f)
		possiblepickups = [locations[int(node)-1] for node in list(roadsOut.keys())]
		possiblepickupsidx = [node for node in list(roadsOut.keys())]
		possibledropoffs = [locations[int(node)-1] for node in list(roadsIn.keys())]
 		possibledropoffsidx = [node for node in list(roadsIn.keys())]
		possiblenodes = [locations[int(node)-1] for node in list(set(roadsOut.keys()) & set(roadsIn.keys()))]
		possiblenodesidx= [node for node in list(set(roadsOut.keys()) & set(roadsIn.keys()))]

		for i, trip in enumerate(reader):
			# if i > count:
			#   break

			hour, minute, second = int(trip[2]), int(trip[3]), int(trip[4])

			if (START_HOUR <= hour < END_HOUR):
				dropoffx, dropoffy = float(trip[7])*1000, float(trip[8])*1000
				pickupx, pickupy = float(trip[5])*1000, float(trip[6])*1000
				closestpickupnode = planparser.findClosestNode(pickupx, pickupy, possiblepickups)
				#closestpickupnode = planparser.findClosestNode(pickupx, pickupy, possiblenodes)
				#print("Pickup distance (short):",planparser.findDistance(pickupx,pickupy,possiblenodes[int(closestpickupnode-1)]))
				#Map the node back to the original list of nodes (not the list of pickupnodes)
				#I do not really understand why we need the -1...Federico
				closestpickupnode = possiblepickupsidx[closestpickupnode-1]
				#closestpickupnode = possiblenodesidx[closestpickupnode-1]
				#print("Pickup distance (long) :",planparser.findDistance(pickupx,pickupy,locations[int(closestpickupnode)-1]))
				listoflinks = roadsOut.get(str(closestpickupnode))
				# Now, what nodes do these links lead to? 
				#Let us only keep the links such that the end node (exit) has neighbors
				#listoflinks = [link for link in listoflinks if len([roadsOut.get(roadMap[link][1])])]
				#listoflinks = [link for link in listoflinks if roadMap[link][1] in possiblenodesidx]
				pickuplink = str(random.choice(listoflinks))

				closestdropoffnode = planparser.findClosestNode(dropoffx, dropoffy, possibledropoffs)
				#closestdropoffnode = planparser.findClosestNode(dropoffx, dropoffy, possiblenodes)
				#print("Dropoff distance (short):",planparser.findDistance(dropoffx,dropoffy,possiblenodes[int(closestdropoffnode-1)]))
				closestdropoffnode = possibledropoffsidx[closestdropoffnode-1]
				#closestdropoffnode = possiblenodesidx[closestdropoffnode-1]
				#print("Dropoff distance (long) :",planparser.findDistance(dropoffx,dropoffy,locations[int(closestdropoffnode)-1]))
				listoflinks = roadsIn.get(str(closestdropoffnode))
				#listoflinks = [link for link in listoflinks if roadMap[link][1] in possiblenodesidx]
				dropofflink = random.choice(listoflinks)

			

				if (dropofflink != pickuplink):
					person = ET.SubElement(plans, "person")
					person.set('id', str(i+1))
					plan = ET.SubElement(person, "plan")
					plan.set('selected', 'yes')
			
					time = datetime.datetime(100,1,1,hour-START_HOUR,minute,second)
					act = ET.SubElement(plan, 'act')
					act.set('type', 'dummy')
					act.set('end_time', str(time.time()))
			
					act.set('link', pickuplink)
					leg = ET.SubElement(plan, 'leg')
					leg.set('mode', 'taxi')
					leg.set('dep_time', str(time.time()))
					leg.set('trav_time', '00:01:00')
					delta = datetime.timedelta(0,TRAV_TIME_SECONDS)
					leg.set('arr_time', str((time + delta).time()))
					#route = ET.SubElement(leg, 'route')
					act2 = ET.SubElement(plan, 'act')
					act2.set('type', 'dummy')
			
					act2.set('link', dropofflink)           
					print(i)

	print("Cleaning XML...")
	parsed = pathwriter.cleanXML(plans)
	parsed = parsed[0:22] + '\n<!DOCTYPE population SYSTEM "http://www.matsim.org/files/dtd/population_v5.dtd">' + \
			 parsed[22:]

	print("Writing XML...")
	outfilename = 'res/march05plansOSM.xml'
	with open(outfilename, 'w') as f:
		f.write(parsed)

def prepRoadData(filename):
	roadsIn = {}
	roadsOut = {}
	roadMap= {}
	with open(filename,'r') as f:
		reader = csv.reader(f)
		for line in reader: 
			start = line[2] #Start and end are actually flipped. Why, Yousef, why?
			end = line[1]
			roadMap[line[0]] = [end, start]
			if roadsIn.get(start) == None:
				roadsIn[start] = [line[0]]
			else:
				roadsIn.get(start).append(line[0])
			if roadsOut.get(end) == None:
				roadsOut[end] = [line[0]]
			else:
				roadsOut.get(end).append(line[0])
	return roadsIn, roadsOut, roadMap

if __name__ == "__main__":
	roadsIn, roadsOut, roadMap = prepRoadData('bin/roadsOSM.csv')
	turnCSVIntoPlans(sys.argv[1], roadsIn, roadsOut, roadMap)
