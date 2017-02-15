import java.util.*;


import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Node;
import org.matsim.contrib.dvrp.data.Vehicle;
import org.matsim.contrib.dvrp.path.VrpPathWithTravelData;
import org.matsim.contrib.dvrp.path.VrpPathWithTravelDataImpl;
import org.matsim.contrib.dvrp.path.VrpPaths;
import org.matsim.contrib.dvrp.schedule.Schedule;
import org.matsim.contrib.dvrp.schedule.Schedules;
import org.matsim.contrib.dvrp.schedule.Schedule.ScheduleStatus;
import org.matsim.contrib.taxi.data.TaxiRequest;
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
 * Class: AMoDSchedulingProblem
 * 
 * This class contains the functionality to dispatch cars
 * and make sure that they are going where they are supposed to be going. 
 * 
 * @author yhindy
 *
 */
public class AMoDSchedulingProblem 
{
	/*  ########################
	 *  ## Instance variables ##
	 *  ########################
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
    
    /** Station-Node data */
	private double[][] stations_to_nodes;
	private double[][] nodes_to_stations;
    
    /**
     * Constructor: AMoDSchedulingProblem
     * ---------------------------------
     * The constructor for this class gives it the data it needs to schedule
     * vehicle trips.
     * 
     * @param optimContext
     * @param vrpFinder
     * @param router
     * @param nodes_to_stations 
     * @param stations_to_nodes 
     */
    public AMoDSchedulingProblem(TaxiOptimizerContext optimContext, BestDispatchFinder vrpFinder, 
    		LeastCostPathCalculator router, double[][] stations_to_nodes, double[][] nodes_to_stations)
    {
        this.optimContext = optimContext;
        this.dispatchFinder = vrpFinder;
        this.router = router;
        this.timer = optimContext.timer;
        this.travelTime = optimContext.travelTime;
        this.stations_to_nodes = stations_to_nodes;
        this.nodes_to_stations = nodes_to_stations;
    }
    
	/**
     * Function findBestPath
     * ---------------------
     * This function finds the best path between two links given a set of possiblepaths and a starttime.
     * The possible paths come from the cplex optimizer. If the cplex optimizer hasn't made a path for the
     * pair of start and end points, then the function creates and returns a path using FastMultiNodeDijkstra. 
     * If the path does exist, the VrpPaths utility will create the path between the two points. 
     * @param from
     * @param to
     * @param possiblepaths
     * @param starttime
     * @return the best path
     */
    private VrpPathWithTravelData findBestPath(Link from, Link to, Set<Path> possiblepaths, double starttime) {
    	if (possiblepaths == null) {
    		return null;
    	}
    	
        for (Path p : possiblepaths) {
        	if (p.links.get(p.links.size()-1).equals(to)) {
        		List<Link> listoflinks = p.links;
        		Link[] links = new Link[listoflinks.size()];
        		double[] linkTTs = new double[links.length];
        		double currTime = starttime;
        		for (int i = 0; i < links.length; i++ ) {
        			Link currlink = listoflinks.get(i);
        			links[i] = currlink;
        			linkTTs[i] = currlink.getLength() / currlink.getFreespeed();
        			currTime += linkTTs[i];
        		}
        		return new VrpPathWithTravelDataImpl(starttime, currTime - starttime, links, linkTTs);
        	}
        } 
        
        return null;
        
    }

    /**
     * Function scheduleOneRequest
     * ----------------------------
     * This function takes in a vehicle, a request, and a set of routes and 
     * alters the vehicle's schedule to pickup the customer. In order to schedule a trip, 
     * five tasks need to be added.
     * 
     * 1. an empty drive task to pickup the passenger
     * 2. a pickup task
     * 3. an occupied drive task to get the customer to the destination
     * 4. a dropoff task
     * 5. a wait task
     * @param v
     * @param req
     * @param currentTime
     * @param fromlink
     * @param toLink
     * @param routes
     * @return 1 or 0 depending on whether or not the route used the cplex route.
     */
    private int scheduleOneRequest(Vehicle v, TaxiRequest req, double currentTime, Link fromlink, Link toLink,
    		Map<Link, Set<Path>> routes) 
    {
    	Schedule<TaxiTask> schedule = TaxiSchedules.asTaxiSchedule(v.getSchedule());
        
        TaxiStayTask lastTask = (TaxiStayTask)Schedules.getLastTask(schedule);
        Set<Path> possiblepaths = routes.get(fromlink);
        
        switch (lastTask.getStatus()) {
        case PLANNED:
            schedule.removeLastTask();// remove waiting
            break;

        case STARTED:
            lastTask.setEndTime(currentTime);// shorten waiting
            break;

        default:
            throw new IllegalStateException();
        }
        
        double t0 = schedule.getStatus() == ScheduleStatus.UNPLANNED ? // see what time it starts
                Math.max(v.getT0(), currentTime) : //
                Schedules.getLastTask(schedule).getEndTime();
                
        VrpPathWithTravelData p1 = VrpPaths.calcAndCreatePath(lastTask.getLink(), fromlink, t0, //path to pickup
                router, travelTime);
        schedule.addTask(new TaxiEmptyDriveTask(p1));
        
        double t1 = p1.getArrivalTime(); //pickup of passenger
        double t2 = t1 + 120;// 2 minutes for picking up the passenger
        schedule.addTask(new TaxiPickupTask(t1, t2, req));
        
        int count = 0;
        VrpPathWithTravelData path = findBestPath(fromlink, toLink, possiblepaths, t2);
        if (path == null) { //path is not in the map lookup
        	path = VrpPaths.calcAndCreatePath(fromlink, toLink, t2, router,
                    travelTime);
        } else {
        	count ++;
        }
        
        schedule.addTask(new TaxiOccupiedDriveTask(path, req)); // add actual drive task
        
        double t3 = path.getArrivalTime();
        double t4 = t3 + 60;// 1 minute for dropping off the passenger
        schedule.addTask(new TaxiDropoffTask(t3, t4,req));
        
        double tEnd = Math.max(t4, v.getT1());
        schedule.addTask(new TaxiStayTask(t4, tEnd, toLink));
        
        return count;
    	
    }

    /**
     * Function scheduleUnplannedRequests
     * ----------------------------------
     * This function goes through all the unplanned requests given to it and plans
     * them all so that each request is matched with a vehicle and is on a vehicle's schedule.
     * 
     * @param unplannedRequests
     * @param routes
     * @param reb_routes
     * @return an array containing the number of cplex routes used and the total number of vehicles scheduled
     */
    public int[] scheduleUnplannedRequests(Queue<TaxiRequest> unplannedRequests, 
    		Map<Link, Set<Path>> routes,
    		Map<Link, Set<Path>> reb_routes)
    {
    	int totalcount = 0;
    	int usedcount = 0;
        while (!unplannedRequests.isEmpty()) {
        	totalcount++;
            TaxiRequest req = unplannedRequests.peek();
            
            double currentTime = timer.getTimeOfDay();

            BestDispatchFinder.Dispatch<TaxiRequest> best = dispatchFinder
                    .findBestVehicleForRequest(req, optimContext.taxiData.getVehicles().values());
            
            if (best == null) {
                return null;
            }
            
            Link fromlink = req.getFromLink();
            Link toLink = req.getToLink();
            
            usedcount += scheduleOneRequest(best.vehicle, req, currentTime, fromlink, toLink, routes);
            
            //TODO search only through available vehicles
            //TODO what about k-nearstvehicle filtering?

          
            //optimContext.scheduler.scheduleRequest(best.vehicle, best.destination, finalfinalpath);
            unplannedRequests.poll();
        }  
       
        int[] stats = new int[]{usedcount, totalcount};
        return stats;
       
    }
    
    /**
     * Function rebalanceVehicles
     * -------------------------------
     * This function takes in a set of rebalancing routes and a percentage
     * to rebalance and then goes through all the idle vehicles and rebalances
     * whatever percentage of them is necessary.
     * 
     * @param reb_routes
     * @param proportion
     * @return the number of free vehicles
     */
	public int rebalanceVehicles(Map<Link, Set<Path>> reb_routes, double proportion) {
		System.out.println("Rebalancing...");
		int count = 0;
		double chance = Math.random();
		for (Vehicle veh : optimContext.taxiData.getVehicles().values()) {
			if (optimContext.scheduler.isIdle(veh) && chance <= proportion) {
				count++;
				rebalanceVehicle(veh, reb_routes);
			} //TODO: figure out why vehicles aren't rebalancing
		}	
		System.out.println(count + " free vehicles.");
		return count;
	}

	/**
	 * Function rebalanceVehicle
	 * --------------------------
	 * This function takes in a vehicle and a set of paths and rebalances the vehicle
	 * according to whether or not a path exists for its location. 
	 * 
	 * @param veh
	 * @param reb_routes
	 */
	private void rebalanceVehicle(Vehicle veh, Map<Link, Set<Path>> reb_routes) {
		Schedule<TaxiTask> curr = TaxiSchedules.asTaxiSchedule(veh.getSchedule());
		TaxiStayTask lasttask = (TaxiStayTask)curr.getCurrentTask();
		Link lastlink = lasttask.getLink();
		Path chosen;
		if (reb_routes.get(lastlink) != null) {
			Set<Path> possiblerebroutes = reb_routes.get(lastlink);
			chosen = chooseRandomPath(possiblerebroutes);
			if (chosen.links.size() == 1 || chosen.nodes.size() == 1) {
				return;
			}
		} else {
			return;
		}
		
		List<Link> links = chosen.links;
		List<Node> nodes = chosen.nodes;
		removeDuplicates(links);
		removeDuplicates(nodes);
		scheduleRebalanceTrip(curr, chosen);
	}

	/**
	 * Function removeDuplicates
	 * -------------------------
	 * This function removes duplicate entries from a list. 
	 * @param listofthings
	 */
	private void removeDuplicates(List<?> listofthings) {
		for (int i = listofthings.size()-1; i > 0; i--) {
			if (listofthings.get(i).equals(listofthings.get(i-1))) {
				listofthings.remove(i);
			}
		}
	}

	/**
	 * Function scheduleRebalanceTrip
	 * ------------------------------
	 * This function takes in a vehicle's schedule and a path and alters
	 * it so that the vehicle will rebalance along the given path. 
	 * After cutting short the last waiting task, it needs to add 2 tasks:
	 * 
	 * 1. an empty drive task
	 * 2. a waiting task
	 * @param curr
	 * @param chosen
	 */
	private void scheduleRebalanceTrip(Schedule<TaxiTask> curr, Path chosen) {
		TaxiStayTask lastTask = (TaxiStayTask)Schedules.getLastTask(curr);
		
		double currentTime = timer.getTimeOfDay();
		
		List<Link> links = chosen.links;
		if (links.size() == 0) {
			return;
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
		Link tolink = links.get(links.size()-1);
		if (links.size() == 0) {
			return;
//			path = VrpPaths.createPath(fromlink, fromlink,
//					currentTime, chosen, travelTime);
		} else {
			links.remove(links.size()-1);
			path = VrpPaths.createPath(fromlink, tolink, 
					currentTime, chosen, travelTime); 
		}
		
		curr.addTask(new AMoDRebalanceTask(path));
		
		double arrivalTime = path.getArrivalTime();
		double tEnd = Math.max(arrivalTime, curr.getVehicle().getT1());
        curr.addTask(new TaxiStayTask(arrivalTime, tEnd, tolink));
	}

	/**
	 * Function chooseRandomPath
	 * --------------------------
	 * This function takes in a set of possible routes and simply 
	 * chooses a random element from the list. 
	 * @param possiblerebroutes
	 * @return
	 */
	private Path chooseRandomPath(Set<Path> possiblerebroutes) {
		int size = possiblerebroutes.size();
		int item = new Random().nextInt(size);
		int i = 0;
		for (Path p : possiblerebroutes) {
			if (i == item) 
				return p;
			i++;
		}
		// shouldn't reach here
		return null;
	}

	/** Station-wise routing */
	public int[] scheduleUnplannedRequestsWithStations(Queue<TaxiRequest> unplannedRequests,
			Map<Pair<Integer, Integer>, Set<Path>> station_routes,
			Map<Pair<Integer, Integer>, Set<Path>> station_reb_routes) {
		int totalcount = 0;
    	int usedcount = 0;
        while (!unplannedRequests.isEmpty()) {
        	totalcount++;
            TaxiRequest req = unplannedRequests.peek();
            
            double currentTime = timer.getTimeOfDay();

            BestDispatchFinder.Dispatch<TaxiRequest> best = dispatchFinder
                    .findBestVehicleForRequest(req, optimContext.taxiData.getVehicles().values());
            
            if (best == null) {
                return null;
            }
            
            Link fromlink = req.getFromLink();
            Link toLink = req.getToLink();
            
            usedcount += scheduleOneRequestWithStations(best.vehicle, req, currentTime, fromlink, toLink, station_routes);
            
            //TODO search only through available vehicles
            //TODO what about k-nearstvehicle filtering?

          
            //optimContext.scheduler.scheduleRequest(best.vehicle, best.destination, finalfinalpath);
            unplannedRequests.poll();
        }  
       
        int[] stats = new int[]{usedcount, totalcount};
        return stats;
	}

	private int scheduleOneRequestWithStations(Vehicle vehicle, TaxiRequest req, double currentTime, Link fromlink,
			Link tolink, Map<Pair<Integer, Integer>, Set<Path>> station_routes)
	{
		// Drive to first node in route outside of station with shortest path computed by Dijkstra
		// Follow the MATLAB path until the next node is in the destination station.
		// Drive to the destination node using Dijkstra. 
		Schedule<TaxiTask> schedule = TaxiSchedules.asTaxiSchedule(vehicle.getSchedule());
        
        TaxiStayTask lastTask = (TaxiStayTask)Schedules.getLastTask(schedule);
        int currVehNode = Integer.parseInt(lastTask.getLink().getFromNode().getId().toString());
        Integer currVehStation = (int) nodes_to_stations[currVehNode - 1][0];
        int startNode = Integer.parseInt(fromlink.getFromNode().getId().toString());
        Integer startStation = (int) nodes_to_stations[startNode-1][0];
        int endNode = Integer.parseInt(tolink.getToNode().getId().toString());
        Integer endStation = (int) nodes_to_stations[endNode-1][0];
        Set<Path> possiblepaths = station_routes.get(new Pair<Integer, Integer>(startStation, endStation));
        
        switch (lastTask.getStatus()) {
        case PLANNED:
            schedule.removeLastTask();// remove waiting
            break;

        case STARTED:
            lastTask.setEndTime(currentTime);// shorten waiting
            break;

        default:
            throw new IllegalStateException();
        }
        
        double t0 = schedule.getStatus() == ScheduleStatus.UNPLANNED ? // see what time it starts
                Math.max(vehicle.getT0(), currentTime) : //
                Schedules.getLastTask(schedule).getEndTime();
        
        VrpPathWithTravelData p1 = VrpPaths.calcAndCreatePath(lastTask.getLink(), fromlink, t0, //path to pickup
                router, travelTime);
        schedule.addTask(new TaxiEmptyDriveTask(p1));
                
        double t1 = p1.getArrivalTime(); //pickup of passenger
        double t2 = t1 + 120;// 2 minutes for picking up the passenger
        schedule.addTask(new TaxiPickupTask(t1, t2, req));
        
        
        Path temppath = findStationPath(fromlink, tolink, possiblepaths);
        int count = 0;
        Node firstOut = findFirstNodeOutOfStation(startStation, temppath);
        
        
		return 0;
	}

	private Node findFirstNodeOutOfStation(Integer startStation, Path temppath) {
		// TODO Auto-generated method stub
		return null;
	}

	private Path findStationPath(Link fromlink, Link tolink, Set<Path> possiblepaths) {
		for (Path p : possiblepaths) {
			if (p.links.get(p.links.size()-1).equals(tolink)) {
				return p;
			}
		}
		return null;
	}

	public int rebalanceVehiclesWithStations(Map<Pair<Integer, Integer>, Set<Path>> station_reb_routes,
			double fraction_to_rebalance) {
		// TODO Auto-generated method stub
		return 0;
	}
}
