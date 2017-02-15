import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Queue;
import java.util.Random;
import java.util.Set;

import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Node;
import org.matsim.contrib.dvrp.data.Vehicle;
import org.matsim.contrib.dvrp.path.VrpPath;
import org.matsim.contrib.dvrp.path.VrpPathWithTravelData;
import org.matsim.contrib.dvrp.path.VrpPathWithTravelDataImpl;
import org.matsim.contrib.dvrp.schedule.DriveTask;
import org.matsim.contrib.dvrp.schedule.Schedule;
import org.matsim.contrib.dvrp.schedule.Schedule.ScheduleStatus;
import org.matsim.contrib.taxi.data.TaxiRequest;
import org.matsim.contrib.taxi.schedule.TaxiSchedules;
import org.matsim.contrib.taxi.schedule.TaxiTask;
import org.matsim.core.router.util.LeastCostPathCalculator.Path;

public class AMoDSchedulingUtils {
	
	/**
	 * Function removeDuplicates
	 * -------------------------
	 * This function removes duplicate entries from a list. 
	 * @param listofthings
	 */
	public static void removeDuplicates(List<?> listofthings) {
		for (int i = listofthings.size()-1; i > 0; i--) {
			if (listofthings.get(i).equals(listofthings.get(i-1))) {
				listofthings.remove(i);
			}
		}
	}
	
	/**
	 * Function chooseRandomPath
	 * --------------------------
	 * This function takes in a set of possible routes and simply 
	 * chooses a random element from the list. 
	 * @param possiblerebroutes
	 * @return
	 */
	public static Object chooseRandomThing(Collection<?> possibleroutes) {
		if (possibleroutes == null) {
			return null;
		}
		int size = possibleroutes.size();
		int item = new Random().nextInt(size);
		int i = 0;
		for (Object p : possibleroutes) {
			if (i == item) 
				return p;
			i++;
		}
		// shouldn't reach here
		return null;
	}

	/**
	 * Function calcTravelTimeFromLinks
	 * --------------------------------
	 * Takes in a list of links and calculates how much time it would take to 
	 * traverse all of them. 
	 * @param links
	 * @return the travel time of the links.
	 */
	public static double calcTravelTimeFromLinks(List<Link> links) {
		double result = 0;
		for (Link l : links) {
			result += l.getFreespeed() / l.getLength();
		}
		return result;
	}

	/**
	 * Function composePathLinks
	 * ------------------------------
	 * This function takes in a start node and an end node and a path that
	 * contains those nodes. It then goes through the path and gets the Links
	 * that connect the two nodes that were given. 
	 * 
	 * @param first
	 * @param last
	 * @param temppath
	 * @return The list of links that connects first and last
	 */
	public static List<Link> composePathLinks(Node first, Node last, Path temppath) {
		List<Link> result = new ArrayList<Link>();
		List<Node> nodes = temppath.nodes;
		List<Link> links = temppath.links;
		int firstIdx = nodes.indexOf(first);
		int lastIdx = nodes.indexOf(last);

		for (int i = firstIdx; i < lastIdx && i >= 0; i++) {
			result.add(links.get(i));
		}
		return result;
	}

	/**
	 * Function mergeLinks
	 * ------------------------
	 * This function takes in three lists of links and a start time and turns
	 * it into a VrpPathWithTravelData so that it can be used
	 * for the TaxiOccupiedDriveTask. 
	 * @param outLinks
	 * @param inBetween
	 * @param inLinks
	 * @param time
	 * @return the smoothed-out path.
	 * @throws InconsistentPathException 
	 */
	public static VrpPathWithTravelData mergeLinks(List<Link> outLinks, List<Link> inBetween, List<Link> inLinks,
			double time) {
		double travelTime = 0;
		
		Link[] finalLinks = new Link[outLinks.size() + inBetween.size() + inLinks.size()];
		Node[] finalnodes = new Node[finalLinks.length + 1];
		int linkIdx = 0;
		
		linkIdx = addSetOfLinks(outLinks, finalLinks, linkIdx, finalnodes);
		
		linkIdx = addSetOfLinks(inBetween, finalLinks, linkIdx, finalnodes);
		
		linkIdx = addSetOfLinks(inLinks, finalLinks, linkIdx, finalnodes);
		
		finalnodes[finalnodes.length - 1] = finalLinks[finalLinks.length - 1].getToNode();
		
		ArrayList<Link> tempLinks = new ArrayList<Link>(Arrays.asList(finalLinks));
		ArrayList<Node> tempNodes = new ArrayList<Node>(Arrays.asList(finalnodes));
		
		finalLinks = fixLoopsInPath(tempLinks, tempNodes); 
		
		double[] linkTTs = new double[finalLinks.length];
		
		travelTime = createTTs(finalLinks, linkTTs);
		
		checkConsistencyOfPath(finalLinks);
		
		VrpPathWithTravelData finalPath = new VrpPathWithTravelDataImpl(time, travelTime, finalLinks, linkTTs);
		return finalPath;
	}
	
	/**
	 * Function: convertNodeListtoPath
	 * -------------------------------------
	 * This function takes two lists: a list of links and a list of nodes. It then
	 * spits them out as a MATSim Path that can be later used by the Controler. 
	 * 
	 * @param nodelist
	 * @param linklist
	 * @return
	 */
	public static Path convertNodeListtoPath(List<Node> nodelist, List<Link> linklist) {
		double traveltime = 0;
		for (Link l : linklist) {
			traveltime += l.getLength()/l.getFreespeed();
		}
		return new Path(nodelist, linklist, traveltime, 0);
	}

	/**
	 * Function: checkConsistencyOfPath:
	 * ---------------------------------
	 * 
	 * This function takes in a series of links and makes sure that they are a consistent
	 * path. A consistent path has link i's toNode = link i+1's fromNode.
	 * 
	 * Mostly used for debugging.
	 * @param finalLinks
	 * @throws InconsistentPathException 
	 */
	private static void checkConsistencyOfPath(Link[] finalLinks) {
		for (int i = 0; i < finalLinks.length - 1; i++) {
			Link curr = finalLinks[i];
			Link next = finalLinks[i+1];
			if (!curr.getToNode().equals(next.getFromNode())) {
				System.out.println("HOUSTON, WE HAVE A PROBLEM WITH THE LINKS.");
			}
		}
		
	}

	/**
	 * Function: createTTs
	 * ------------------
	 * 
	 * This function takes in a series of links and an empty array
	 * and fills the array with the travel times of each link. It then
	 * returns the total travel time as a double.
	 * 
	 * @param finalLinks
	 * @param linkTTs
	 * @return the total travel time
	 */
	private static double createTTs(Link[] finalLinks, double[] linkTTs) {
		double total = 0;
		for (int i = 0; i < finalLinks.length; i++) {
			Link curr = finalLinks[i];
			linkTTs[i] = curr.getFreespeed() / curr.getLength();
			total += linkTTs[i];
		}
		return total;
	}

	/**
	 * Function fixLoopsInPath
	 * -----------------------
	 * This function determines whether or not a path needs to be fixed.
	 * A path needs to be fixed if it goes through the same node twice.
	 * If it needs to be fixed, the path is passed to another function for fixing.
	 * 
	 * NB: Doesn't fix loops if the node is the first node of the path, as MATSim
	 * requires that the first link be the same as the Taxi Request.
	 * 
	 * @param tempLinks: list of links to be fixed.
	 * @param tempNodes: list of nodes to be fixed.
	 * @return the fixed path. 
	 */
	private static Link[] fixLoopsInPath(ArrayList<Link> tempLinks, ArrayList<Node> tempNodes) {
		for (int i = 0; i < tempNodes.size(); i++) {
			Node n = tempNodes.get(i);
			int ind = tempNodes.indexOf(n);
			if (ind != tempNodes.size() - 1 && ind != 0) {
				List<Node> sub = tempNodes.subList(ind+1, tempNodes.size());
				int ind2 = sub.indexOf(n);
				if (ind2 != -1  && (ind2 + ind + 1) != tempNodes.size() - 1) { // TODO: figure out a better way to do this
					ind2 += ind + 1;
					fixLoop(tempLinks, tempNodes, ind, ind2);
				}
			}
		}
		
		removeDuplicates(tempLinks);
		Link[] finalLinks = new Link[tempLinks.size()];
		tempLinks.toArray(finalLinks);
		return finalLinks;
	}

	/**
	 * Function fixLoop
	 * ----------------
	 * This function takes in a series of links, a series of nodes, and a start and end index.
	 * It then goes through the links and takes out the necessary links so that after 
	 * the function is done, the path does not contain that particular loop.
	 * 
	 * @param tempLinks
	 * @param tempNodes
	 * @param ind
	 * @param ind2
	 */
	private static void fixLoop(ArrayList<Link> tempLinks, ArrayList<Node> tempNodes, int ind, int ind2) {
		for (int i = ind2 - 1; i >= ind; i--) {
			tempLinks.remove(i);
			tempNodes.remove(i);
		}
		
		removeDuplicates(tempLinks);
	}

	/**
	 * Function addSetOfLinks
	 * ----------------------
	 * This function takes a list of links and appends it to the end of a final list of links,
	 * updating the travel times accordingly. 
	 * 
	 * @param links: links to be added
	 * @param finalLinks: list to which links are added
	 * @param linkTTs: travelTime array of links
	 * @param travelTime: total travelTime
	 * @param linkIdx: the starting index to add elements
	 * @return linkIdx: where to start the next iteration
	 */
	private static int addSetOfLinks(List<Link> links, Link[] finalLinks,
			int linkIdx, Node[] finalNodes) {
		
		for (int i = 0; i < links.size(); i++) {
			Link l = links.get(i);
			finalLinks[linkIdx] = l;
			finalNodes[linkIdx] = l.getFromNode();
			linkIdx++;
		}
		return linkIdx;
	}

	/**
	 * Function calculateVehicleLocations
	 * ----------------------------------
	 * This function takes in a collection of vehicles, a number of links, and a simulation time
	 * and returns an array of how many vehicles are on each link. The ith entry of the array represents
	 * the number of vehicles at time simTime that are on link i + 1. 
	 * @param vehicles
	 * @param numLinks
	 * @param simTime
	 * @return
	 */
	public static double[] calculateVehicleLocations(Collection<Vehicle> vehicles, int numLinks, double simTime) {
		double[] result = new double[numLinks];
		for (Vehicle v : vehicles)  {
			Schedule<TaxiTask> sched = TaxiSchedules.asTaxiSchedule(v.getSchedule());
			if (sched.getStatus() == ScheduleStatus.STARTED) {
				TaxiTask curr = sched.getCurrentTask();
				if (curr instanceof DriveTask) {
					VrpPath path = ((DriveTask) curr).getPath();
					double driveTime = curr.getBeginTime();
					int linkIdx = 0; 
					while (driveTime <= simTime && linkIdx < path.getLinkCount()) { // figuring out where the current link is
						driveTime += path.getLinkTravelTime(linkIdx);
						linkIdx++;
					}
					if (linkIdx != 0) {
						result[Integer.parseInt(path.getLink(linkIdx-1).getId().toString())-1]++;
					}	
				}
			}
		}
		return result;
	}

	public static double[] calculateNumberWaitingPassengers(Collection<TaxiRequest> unplannedRequests,
			double[][] nodes_to_stations, double[][] stations_to_nodes) {
		double[] result = new double[stations_to_nodes.length];
		
		for (TaxiRequest req : unplannedRequests) {
			Link fromlink = req.getFromLink();
			Node start = fromlink.getFromNode();
			int nodeId = Integer.parseInt(start.getId().toString());
			int station = (int) nodes_to_stations[nodeId-1][0];
			result[station-1]++;
		}
		
		return result;
	}
}
