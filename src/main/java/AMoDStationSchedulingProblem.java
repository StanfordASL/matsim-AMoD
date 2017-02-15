import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.*;

import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Node;
import org.matsim.contrib.dvrp.data.Vehicle;
import org.matsim.contrib.dvrp.path.VrpPathWithTravelData;
import org.matsim.contrib.dvrp.path.VrpPaths;
import org.matsim.contrib.dvrp.schedule.Schedule;
import org.matsim.contrib.dvrp.schedule.Schedules;
import org.matsim.contrib.dvrp.schedule.Tasks;
import org.matsim.contrib.dvrp.schedule.Schedule.ScheduleStatus;
import org.matsim.contrib.taxi.data.TaxiRequest;
import org.matsim.contrib.taxi.data.TaxiRequest.TaxiRequestStatus;
import org.matsim.contrib.taxi.optimizer.*;
import org.matsim.contrib.taxi.schedule.TaxiDropoffTask;
import org.matsim.contrib.taxi.schedule.TaxiEmptyDriveTask;
import org.matsim.contrib.taxi.schedule.TaxiOccupiedDriveTask;
import org.matsim.contrib.taxi.schedule.TaxiPickupTask;
import org.matsim.contrib.taxi.schedule.TaxiSchedules;
import org.matsim.contrib.taxi.schedule.TaxiStayTask;
import org.matsim.contrib.taxi.schedule.TaxiTask;
import org.matsim.core.mobsim.framework.MobsimTimer;
import org.matsim.core.router.util.LeastCostPathCalculator;
import org.matsim.core.router.util.LeastCostPathCalculator.Path;
import org.matsim.core.router.util.TravelTime;

/**
 * Class: AMoDStationSchedulingProblem
 * 
 * This class contains the functionality to dispatch cars and make sure that
 * they are going where they are supposed to be going. STATION-WISE
 * 
 * @author yhindy
 *
 */
public class AMoDStationSchedulingProblem {
	/*
	 * ######################## ## Instance variables ##
	 * ########################
	 */

	/** The context of the simulation */
	private final TaxiOptimizerContext optimContext;
	/** The dispatcher to find the best vehicle */
	private final BestDispatchFinder dispatchFinder;
	/** The router to find routes in case cplex hasn't made one */
	private final LeastCostPathCalculator router;
	/** Timer of the simulation */
	private final MobsimTimer timer;
	/** TravelTime calculator */
	private final TravelTime travelTime;
	/** Scheduler */
	private final AMoDTaxiScheduler scheduler;


	/** Station-Node data */
	private double[][] nodes_to_stations;
	private double[][] stations_to_nodes;

	/**
	 * Constructor: AMoDSchedulingProblem --------------------------------- The
	 * constructor for this class gives it the data it needs to schedule vehicle
	 * trips.
	 * 
	 * @param optimContext
	 * @param vrpFinder
	 * @param router
	 * @param nodes_to_stations
	 * @param stations_to_nodes
	 * @param stations_to_nodes
	 */
	public AMoDStationSchedulingProblem(TaxiOptimizerContext optimContext, BestDispatchFinder vrpFinder,
			LeastCostPathCalculator router, double[][] nodes_to_stations, double[][] stations_to_nodes) {
		this.optimContext = optimContext;
		this.dispatchFinder = vrpFinder;
		this.router = router;
		this.timer = optimContext.timer;
		this.travelTime = optimContext.travelTime;
		this.nodes_to_stations = nodes_to_stations;
		this.stations_to_nodes = stations_to_nodes;
		this.scheduler = (AMoDTaxiScheduler) optimContext.scheduler;
	}

	/**
	 * Function scheduleUnplannedRequests ----------------------------------
	 * This function goes through all the unplanned requests given to it and
	 * plans them all so that each request is matched with a vehicle and is on a
	 * vehicle's schedule.
	 * 
	 * @param unplannedRequests
	 * @param routes
	 * @param reb_routes
	 * @return an array containing the number of cplex routes used and the total
	 *         number of vehicles scheduled
	 * @throws UnsupportedEncodingException
	 * @throws FileNotFoundException
	 */
	public Set<TaxiRequest> scheduleUnplannedRequests(Queue<TaxiRequest> unplannedRequests,
			Map<Integer, Map<Integer, Set<Path>>> station_routes)
			throws FileNotFoundException, UnsupportedEncodingException {
		int totalcount = 0;
		int usedcount = 0;
		// PrintWriter writer = new PrintWriter("requests_stations1.txt",
		// "UTF-8");
		Set<TaxiRequest> impossibleReqs = new HashSet<TaxiRequest>();
		while(!unplannedRequests.isEmpty()) {
			totalcount++;
			TaxiRequest req = unplannedRequests.poll();
			double currentTime = timer.getTimeOfDay();

			BestDispatchFinder.Dispatch<TaxiRequest> best = dispatchFinder.findBestVehicleForRequest(req,
					optimContext.taxiData.getVehicles().values());

			if (best != null) {
				Link fromlink = req.getFromLink();
				Link toLink = req.getToLink();

				// best.path is the path to the pickup

				usedcount += scheduleOneRequest(best.vehicle, req, best.path, best.path.getDepartureTime(), fromlink,
						toLink, station_routes);

				// TODO search only through available vehicles
				// TODO what about k-nearstvehicle filtering?

				// optimContext.scheduler.scheduleRequest(best.vehicle,
				// best.destination, finalfinalpath);
			} else {
				impossibleReqs.add(req);
			}
		}
		// writer.close();
		
		return impossibleReqs;
	}

	/**
	 * Function scheduleOneRequest ---------------------------- This function
	 * takes in a vehicle, a request, and a set of routes and alters the
	 * vehicle's schedule to pickup the customer. In order to schedule a trip,
	 * five tasks need to be added.
	 * 
	 * 1. an empty drive task to pickup the passenger 2. a pickup task 3. an
	 * occupied drive task to get the customer to the destination 4. a dropoff
	 * task 5. a wait task
	 * 
	 * @param v
	 * @param req
	 * @param currentTime
	 * @param fromlink
	 * @param toLink
	 * @param station_routes
	 * @return 1 or 0 depending on whether or not the route used the cplex
	 *         route.
	 */
	private int scheduleOneRequest(Vehicle v, TaxiRequest req, VrpPathWithTravelData toPickup, double startTime,
			Link fromlink, Link tolink, Map<Integer, Map<Integer, Set<Path>>> station_routes) {
		// Drive to first node in route outside of station with shortest path
		// computed by Dijkstra
		// Follow the MATLAB path until the next node is in the destination
		// station.
		// Drive to the destination node using Dijkstra.

		if (req.getStatus() != TaxiRequestStatus.UNPLANNED) {
			throw new IllegalStateException();
		}

		Schedule<TaxiTask> schedule = TaxiSchedules.asTaxiSchedule(v.getSchedule());

		int startNode = Integer.parseInt(fromlink.getToNode().getId().toString());
		int startStation = (int) nodes_to_stations[startNode - 1][0];
		int endNode = Integer.parseInt(tolink.getFromNode().getId().toString());
		int endStation = (int) nodes_to_stations[endNode - 1][0];
		Set<Path> possiblepaths;
		Map<Integer, Set<Path>> outOfStart = null;
		if (station_routes == null) {
			possiblepaths = null;
		} else {
			outOfStart = station_routes.get(startStation);
		}

		if (outOfStart == null) {
			possiblepaths = null;
		} else {
			possiblepaths = outOfStart.get(endStation);
		}

		scheduler.scheduleFirstHalf(v, req, toPickup);
		double t3 = scheduler.calcJourneyStartTime(schedule);
		int count = 0;
		Path temppath = (Path) AMoDSchedulingUtils.chooseRandomThing(possiblepaths);
		VrpPathWithTravelData p2;
		if (temppath == null) {
			p2 = VrpPaths.calcAndCreatePath(fromlink, tolink, t3, router, travelTime);
		} else {
			p2 = findPatchedPath(temppath, fromlink, tolink, startStation, endStation, t3);
			count++;

		}

		scheduler.scheduleSecondHalf(v, req, p2);

		// p2 is now the main path of the journey

		return count;
	}

	/**
	 * Function findPatchedPath ---------------------------- This function takes
	 * in a path, a fromlink, tolink, startStation, and endStation and patches
	 * the path so that the car travels to the first node outside of the
	 * station, follows the MATLAB path, then goes straight from the last node
	 * outside of the end station to the destination.
	 * 
	 * Each path consists of three parts: 1. outLinks: these are the links out
	 * from the origin to the first node outside of the current station 2.
	 * middleLinks: these are the links that are MATLAB generated that take the
	 * car from start station to end station. 3. inLinks: these are the links
	 * into the final station that take the car from the MATLAB path to the
	 * passenger's destination.
	 * 
	 * These three parts are optimally combined to reduce any extra driving.
	 * 
	 * @param temppath:
	 *            the MATLAB path
	 * @param fromlink:
	 *            startlink of total journey
	 * @param tolink:
	 *            endlink of total journey
	 * @param startStation:
	 *            startStation
	 * @param endStation:
	 *            endStation
	 * @param time:
	 *            starttime of destination
	 * @return the patched path
	 */
	private VrpPathWithTravelData findPatchedPath(Path temppath, Link fromlink, Link tolink, Integer startStation,
			Integer endStation, double time) {

		List<Node> nodes = temppath.nodes;

		Node firstOut = findFirstToNode(nodes, startStation);
		Path outOfStation;
		if (firstOut == null) { // always in a single station
			outOfStation = router.calcLeastCostPath(fromlink.getToNode(), nodes.get(0), time, null, null);
		} else {
			outOfStation = router.calcLeastCostPath(fromlink.getToNode(), firstOut, time, null, null);
		}
		List<Link> outLinks = outOfStation.links;
		outLinks.add(0, fromlink);
		Node last = findLastNonStationNode(nodes, endStation);

		Path inBetween;
		List<Link> middleLinks;
		if (last == null) { // need to make middleLinks the same as temppath
			inBetween = temppath;
			middleLinks = inBetween.links;
			last = middleLinks.get(middleLinks.size() - 1).getToNode();
		} else {
			middleLinks = AMoDSchedulingUtils.composePathLinks(firstOut, last, temppath);
		}

		double inBetweenTime = 0;
		Path intoStation;
		if (middleLinks.size() == 0) { // something is wonky (stations could be
										// adjacent, in which case nodes are all
										// messed up)
			intoStation = router.calcLeastCostPath(outLinks.get(outLinks.size() - 1).getToNode(), tolink.getFromNode(),
					time + outOfStation.travelTime, null, null);
		} else {
			inBetweenTime = AMoDSchedulingUtils.calcTravelTimeFromLinks(middleLinks);
			intoStation = router.calcLeastCostPath(last, tolink.getFromNode(),
					time + outOfStation.travelTime + inBetweenTime, null, null);
		}
		List<Link> inLinks = intoStation.links;
		inLinks.add(tolink);

		VrpPathWithTravelData finalpath = AMoDSchedulingUtils.mergeLinks(outLinks, middleLinks, inLinks, time);
		return finalpath;
	}

	/**
	 * Function findFirstToNode ------------------------- This function takes in
	 * a list of Nodes and a station and finds the first node in the path that
	 * is outside of the starting station.
	 * 
	 * @param path
	 * @param startStation
	 * @return
	 */
	private Node findFirstToNode(List<Node> path, int startStation) {
		for (Node n : path) {
			int nodenumber = Integer.parseInt(n.getId().toString());
			if ((int) nodes_to_stations[nodenumber - 1][0] != startStation) {
				return n;
			}
		}
		return null;
	}

	/**
	 * Function findLastNonStationNode ------------------------------ This
	 * function finds the last node in a path that is not in endStation.
	 * 
	 * @param path
	 * @param endStation
	 * @return the first Node outside of the endStation.
	 */
	private Node findLastNonStationNode(List<Node> path, Integer endStation) {
		for (int i = path.size() - 1; i >= 0; i--) {
			Node curr = path.get(i);
			int currnumber = Integer.parseInt(curr.getId().toString());
			if ((int) nodes_to_stations[currnumber - 1][0] != endStation) {
				return curr;
			}
		}
		return null;
	}

	/**
	 * Function legacyRebalanceVehicles -------------------------------- This
	 * function takes in a set of rebalancing origins/destinations and a
	 * percentage to rebalance and then goes through all the idle vehicles and
	 * rebalances whatever percentage is necessary using Dijkstra.
	 * 
	 * @param rebalance_queue
	 * @param fraction_to_rebalance
	 * @return number of free vehicles
	 */
	public int legacyRebalanceVehicles(List<List<Integer>> rebalance_queue, double fraction_to_rebalance) {
		int count = 0;
		for (Vehicle veh : optimContext.taxiData.getVehicles().values()) {
			if (optimContext.scheduler.isIdle(veh)) {
				count += legacyRebalanceVehicle(veh, rebalance_queue, fraction_to_rebalance);
			}
		}
		return count;
	}

	/**
	 * Function legacyRebalanceVehicle
	 * 
	 * This function takes in a vehicle to rebalance, a list of station pairs,
	 * and a fraction_to_rebalance and rebalances a certain vehicle by checking
	 * what station it is in and then seeing which stations need to be
	 * rebalanced to. MATSim then uses Dijsktra to calculate the path
	 * 
	 * @param veh
	 * @param rebalance_queue
	 * @param fraction_to_rebalance
	 * @return
	 */
	private int legacyRebalanceVehicle(Vehicle veh, List<List<Integer>> rebalance_queue, double fraction_to_rebalance) {
		if (rebalance_queue == null) {
			return 0;
		}
		Schedule<TaxiTask> curr = TaxiSchedules.asTaxiSchedule(veh.getSchedule());
		TaxiStayTask lasttask = (TaxiStayTask) curr.getCurrentTask();
		Link lastlink = lasttask.getLink();
		Node lastnode = lastlink.getToNode();
		int lastnodenumber = Integer.parseInt(lastnode.getId().toString());
		Integer laststation = (int) nodes_to_stations[lastnodenumber - 1][0];
		Path chosen;
		double chance = Math.random();
		if (!rebalance_queue.get(laststation - 1).isEmpty()) {
			List<Integer> possibleDestStations = rebalance_queue.get(laststation - 1);
			Integer destination = (Integer) AMoDSchedulingUtils.chooseRandomThing(possibleDestStations);
			possibleDestStations.remove(destination); // spending the trip
			String destNodeId = Integer.toString((int) stations_to_nodes[destination - 1][0]);
			Node destNode = optimContext.network.getNodes().get(Id.createNodeId(destNodeId));
			chosen = router.calcLeastCostPath(lastnode, destNode, timer.getTimeOfDay(), null, null);
		} else {
			return 0;
		}

		if (chosen == null) {
			return 0;
		}
		List<Link> links = chosen.links;
		List<Node> nodes = chosen.nodes;
		AMoDSchedulingUtils.removeDuplicates(links);
		AMoDSchedulingUtils.removeDuplicates(nodes);
		return scheduleRebalanceTrip(curr, chosen);
	}

	/**
	 * Function rebalanceVehicles ------------------------------- This function
	 * takes in a set of rebalancing routes and a percentage to rebalance and
	 * then goes through all the idle vehicles and rebalances whatever
	 * percentage of them is necessary.
	 * 
	 * @param station_reb_routes
	 * @param proportion
	 * @return the number of free vehicles
	 */
	public int rebalanceVehicles(Map<Integer, List<Path>> station_reb_routes, double proportion) {
		if (station_reb_routes == null) {
			return 0;
		}
		// System.out.println("Rebalancing...");
		int count = 0;
		for (Vehicle veh : optimContext.taxiData.getVehicles().values()) {
			if (optimContext.scheduler.isIdle(veh)) {
				count += rebalanceVehicle(veh, station_reb_routes, proportion);
			}
		}
		return count;
	}

	/**
	 * Function rebalanceVehicle ------------------------------ This function
	 * takes in a vehicle and a set of paths and rebalances the vehicle
	 * according to whether or not a path exists for its location.
	 * 
	 * @param veh
	 * @param station_reb_routes
	 * @param proportion
	 */
	private int rebalanceVehicle(Vehicle veh, Map<Integer, List<Path>> station_reb_routes, double proportion) {
		Schedule<TaxiTask> curr = TaxiSchedules.asTaxiSchedule(veh.getSchedule());
		TaxiStayTask lasttask = (TaxiStayTask) curr.getCurrentTask();
		Link lastlink = lasttask.getLink();
		Node lastnode = lastlink.getToNode();
		int lastnodenumber = Integer.parseInt(lastnode.getId().toString());
		Integer laststation = (int) nodes_to_stations[lastnodenumber - 1][0];
		Path chosen;
		if (station_reb_routes.get(laststation) != null) {
			List<Path> possiblerebroutes = station_reb_routes.get(laststation);
			if (possiblerebroutes.size() == 0) {
				station_reb_routes.remove(laststation);
				return 0; 
			}
			Path chosen2 = (Path) AMoDSchedulingUtils.chooseRandomThing(possiblerebroutes);
			if ( chosen2.links.size() == 1 || chosen2.nodes.size() == 1) {
				return 0;
			}
			
			chosen = patchRebalanceTrip(lastnode, chosen2);
			possiblerebroutes.remove(chosen2);
		} else {
			return 0;
		}

		if (chosen == null) {
			return 0;
		}
		List<Link> links = chosen.links;
		List<Node> nodes = chosen.nodes;
		AMoDSchedulingUtils.removeDuplicates(links);
		AMoDSchedulingUtils.removeDuplicates(nodes);
		return scheduleRebalanceTrip(curr, chosen);
	}

	/**
	 * Function patchRebalanceTrip ---------------------------- This function
	 * takes in a node and a path and creates a Path that contains the path from
	 * lastnode to the first node of chosen and then the links in chosen.
	 * 
	 * @param lastnode
	 * @param chosen
	 * @return
	 */
	private Path patchRebalanceTrip(Node lastnode, Path chosen) {
		Node firstNode = chosen.nodes.get(0);
		Path patch = router.calcLeastCostPath(lastnode, firstNode, timer.getTimeOfDay(), null, null);
		List<Link> patchlinks = patch.links;
		List<Node> patchnodes = patch.nodes;
		List<Link> chosenlinks = chosen.links;
		List<Node> chosennodes = chosen.nodes;
		int ind = chosennodes.indexOf(lastnode);
		if (ind >= 0) { // the node the car is on is already on the path
			double extraTime = 0;
			for (int i = 0; i < ind && i < chosenlinks.size(); i++) {
				extraTime += chosenlinks.get(i).getFreespeed() / chosenlinks.get(i).getLength();
			}
			if (ind > chosenlinks.size()) { // car is already at the end of the
											// path
				return null;
			}
			return new LeastCostPathCalculator.Path(chosennodes.subList(ind, chosennodes.size()),
					chosenlinks.subList(ind, chosenlinks.size()), chosen.travelTime - extraTime, 0);
		} else { //
			for (int i = 0; i < chosenlinks.size(); i++) {
				patchlinks.add(chosenlinks.get(i));
				patchnodes.add(chosennodes.get(i + 1));
			}
			return new LeastCostPathCalculator.Path(patchnodes, patchlinks, chosen.travelTime + patch.travelTime, 0);
		}
	}

	/**
	 * Function scheduleRebalanceTrip ------------------------------ This
	 * function takes in a vehicle's schedule and a path and alters it so that
	 * the vehicle will rebalance along the given path. After cutting short the
	 * last waiting task, it needs to add 2 tasks:
	 * 
	 * 1. an empty drive task 2. a waiting task
	 * 
	 * @param curr
	 * @param chosen
	 */
	private int scheduleRebalanceTrip(Schedule<TaxiTask> curr, Path chosen) {
		TaxiStayTask lastTask = (TaxiStayTask) Schedules.getLastTask(curr);

		double currentTime = timer.getTimeOfDay();

		List<Link> links = chosen.links;
		if (links.size() == 0) {
			return 0;
		}

		switch (lastTask.getStatus()) {
		case PLANNED:
			curr.removeLastTask();// remove waiting
			break;

		case STARTED:
			lastTask.setEndTime(currentTime);// shorten waiting
			break;

		default:
			throw new IllegalStateException();
		}

		Link fromlink = lastTask.getLink();

		Link potentialstart = links.get(0);
		if (potentialstart.equals(fromlink)) {
			links.remove(0);
		}
		VrpPathWithTravelData path;

		Link tolink;
		if (links.size() == 0) {
			return 0;
		} else {
			tolink = links.get(links.size() - 1);
			links.remove(links.size() - 1);
			path = VrpPaths.createPath(fromlink, tolink, currentTime, chosen, travelTime);
		}

		curr.addTask(new AMoDRebalanceTask(path));

		double arrivalTime = path.getArrivalTime();
		double tEnd = Math.max(arrivalTime, curr.getVehicle().getT1());
		curr.addTask(new TaxiStayTask(arrivalTime, tEnd, tolink));
		return 1;
	}

}
