import xml.etree.ElementTree as ET
from xml.etree import ElementTree
from xml.dom import minidom
import scipy.io as sio
import pathwriter
import planparser
import random

NUMLINKS = 3137
NUMVEHICLES = 8000
SEED = 4711
t0 = 0
t1 = 2*60*60 + .5*60*60

def initializeVehicle(vehicles, num):
	vehicle = ET.SubElement(vehicles, 'vehicle')
	vehicle.set('id', 'amod_{}'.format(str(num)))
	startlink = random.randint(1, NUMLINKS)
	vehicle.set('start_link', str(startlink))
	vehicle.set('t_0', str(t0))
	vehicle.set('t_1', str(t1))

def makeVehicles():
	vehicles = ET.Element('vehicles')
	for i in range(NUMVEHICLES):
		initializeVehicle(vehicles, i)

	parsed = pathwriter.cleanXML(vehicles)

	parsed = parsed[0:22] + '\n<!DOCTYPE vehicles SYSTEM "http://www.matsim.org/files/dtd/dvrp_vehicles_v1.dtd">' + \
			 parsed[22:]

	outfile = 'res/amod_vehicles_OSM.xml'
	with open(outfile, 'w') as f:
		f.write(parsed)

if __name__ == '__main__':
	makeVehicles()
