import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.*;

import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.Node;
import org.matsim.contrib.dvrp.data.Requests;
import org.matsim.contrib.dvrp.data.Vehicle;
import org.matsim.contrib.dvrp.schedule.Schedule;
import org.matsim.contrib.dvrp.schedule.Schedules;
import org.matsim.contrib.dvrp.schedule.Schedule.ScheduleStatus;
import org.matsim.contrib.dvrp.schedule.Task;
import org.matsim.contrib.taxi.data.TaxiRequest;
import org.matsim.contrib.taxi.optimizer.*;
import org.matsim.contrib.taxi.schedule.TaxiSchedules;
import org.matsim.contrib.taxi.schedule.TaxiStayTask;
import org.matsim.contrib.taxi.schedule.TaxiTask;
import org.matsim.core.mobsim.framework.events.MobsimBeforeSimStepEvent;
import org.matsim.core.router.ArrayFastRouterDelegateFactory;
import org.matsim.core.router.FastMultiNodeDijkstra;
import org.matsim.core.router.FastRouterDelegateFactory;
import org.matsim.core.router.util.ArrayRoutingNetworkFactory;
import org.matsim.core.router.util.LeastCostPathCalculator;
import org.matsim.core.router.util.PreProcessDijkstra;
import org.matsim.core.router.util.RoutingNetwork;
import org.matsim.core.router.util.LeastCostPathCalculator.Path;
import matlabcontrol.MatlabConnectionException;
import matlabcontrol.MatlabInvocationException;
import matlabcontrol.MatlabProxy;

import com.jmatio.io.MatFileReader;
import com.jmatio.io.MatFileWriter;
import com.jmatio.types.MLArray;
import com.jmatio.types.MLCell;
import com.jmatio.types.MLDouble;

/**
 * Class: AMoDTaxiOptimizer.java
 * 
 * This file contains the functionality to route the vehicle agents of the
 * simulation according to the MATLAB algorithms used in the
 * CDC16_code_workflow.m file.
 * 
 * @author yhindy
 */

public class AMoDTaxiOptimizer extends AbstractTaxiOptimizer {

	/*
	 * ######################## ## Instance variables ##
	 * ########################
	 */

	/** These are used to hold route data */
	private Map<Link, Set<Path>> routes; // maps origins to a map connecting
											// destinations to routes.
	private Map<Link, Set<Path>> reb_routes; // rebalancing routes
	private Map<Link, Set<Path>> new_routes; // routes received by the
												// optimizer, aren't used right
												// away
	private Map<Link, Set<Path>> new_reb_routes; // these become "routes" once
													// optimizerDelay has passed
	private List<List<Integer>> rebalance_queue;

	/**
	 * These are used to hold route data station-wise\ They map a pair of
	 * integers (start and end stations) to paths that connect them.
	 */
	private Map<Integer, Map<Integer, Set<Path>>> station_routes = new HashMap<Integer, Map<Integer, Set<Path>>>();
	private Map<Integer, List<Path>> station_reb_routes = new HashMap<Integer, List<Path>>();
	private Map<Integer, Map<Integer, Set<Path>>> new_station_routes;
	private Map<Integer, List<Path>> new_station_reb_routes;

	/**
	 * This set holds requests that couldn't be made because there wasn't a car
	 * there
	 */
	private Set<TaxiRequest> rolloverRequests;

	/** These are used by the optimizer to find routes and cars */
	private final BestDispatchFinder dispatchFinder;
	private LeastCostPathCalculator router;
	private MatlabProxy proxy;
	double[][] stations_to_nodes;
	double[][] nodes_to_stations;

	/** These are set by the optimizer group of the config-group. */
	private int timeHorizon;
	private int optimizerDelay;
	private int optimizerEndTime = 0;
	private double rebWeight;
	private int reoptimizationTimeStep;
	private AMoDPerformance stats;
	private String optimizerDataFile;
	private double tripThreshold;
	private boolean vehicleDiversion = true;
	private String stationMap;
	private boolean legacyRebalance;
	private boolean doUnscheduleAwaitingRequests;
	private int reschedulePeriod; // how long to wait before putting waiting
									// customers back into the queue
	private boolean amodDispatch;

	/** Flag to use station-wise routing */
	private boolean use_stations = true;

	double fraction_to_rebalance;

	private int simTime;

	/*
	 * ########################## #### Member functions ####
	 * ##########################
	 */

	/**
	 * Constructor: AMoDTaxiOptimizer
	 * ------------------------------------------------------ Creates an
	 * instance of the AMoDTaxiOptimizer
	 * 
	 * @param optimContext:
	 *            the context of the simulation
	 * @param params:
	 *            the parameters necessary for AMoD
	 * @throws MatlabConnectionException:
	 *             this occurs if the connection to MATLAB cannot be
	 *             established.
	 * @throws IOException
	 * @throws FileNotFoundException
	 */
	public AMoDTaxiOptimizer(TaxiOptimizerContext optimContext, AMoDTaxiOptimizerParams params)
			throws MatlabConnectionException, FileNotFoundException, IOException {
		super(optimContext, params, new PriorityQueue<TaxiRequest>(100, Requests.T0_COMPARATOR), true);

		/* Setting instance variables from parameters of XML file */
		optimizerDelay = params.optimizerDelay;
		rebWeight = params.rebWeight;
		reoptimizationTimeStep = params.reoptimizationTimeStep;
		timeHorizon = params.timeHorizon;
		optimizerDataFile = params.optimizerDataFile;
		tripThreshold = params.tripThreshold;
		stationMap = params.stationMap;
		legacyRebalance = params.legacyRebalance;
		reschedulePeriod = params.reschedulePeriod;
		amodDispatch = params.amodDispatch;
		rolloverRequests = new HashSet<TaxiRequest>();

		double[][] linkMap = new double[2][optimContext.network.getLinks().values().size()];

		writelinkMap(linkMap, optimContext.network.getLinks().values());

		if (use_stations) {
			station_routes = new HashMap<Integer, Map<Integer, Set<Path>>>();
			station_reb_routes = new HashMap<Integer, List<Path>>();
		} else {
			routes = new HashMap<Link, Set<Path>>();
			reb_routes = new HashMap<Link, Set<Path>>();
		}

		/* Creating an object to track performance of the simulation */
		stats = new AMoDPerformance(optimContext);

		initMatlab();

		if (amodDispatch) {
			dispatchFinder = new AMoDDispatchFinder(optimContext, params.neighbourhoodSize, stations_to_nodes,
					nodes_to_stations);
		} else {
			dispatchFinder = new BestDispatchFinder(optimContext, params.neighbourhoodSize);
		}

		PreProcessDijkstra preProcessDijkstra = null;
		FastRouterDelegateFactory fastRouterFactory = new ArrayFastRouterDelegateFactory();

		/*
		 * Building a router for the case that a path cannot be found between
		 * two points by the MATLAB optimizer
		 */
		RoutingNetwork routingNetwork = new ArrayRoutingNetworkFactory(preProcessDijkstra)
				.createRoutingNetwork(optimContext.network);
		router = new FastMultiNodeDijkstra(routingNetwork, optimContext.travelDisutility, optimContext.travelTime,
				preProcessDijkstra, fastRouterFactory, false);

	}

	private void writelinkMap(double[][] linkMap, Collection<? extends Link> values) throws IOException {
		int i = 0;
		for (Link l : values) {
			linkMap[0][i] = Integer.parseInt(l.getFromNode().getId().toString());
			linkMap[1][i] = Integer.parseInt(l.getToNode().getId().toString());
			i++;
		}

		ArrayList<MLArray> list = new ArrayList<MLArray>();
		MLDouble matlinkMap = new MLDouble("linkMap", linkMap);
		list.add(matlinkMap);

		new MatFileWriter("MATLAB_utils/linkMap.mat", list);
	}

	/**
	 * Function callCPLEXOptimizer ------------------------------------- This
	 * function calls the CPLEX optimizer inside of the runOptimization.m file,
	 * which should be located in /AMoD_taxi/MATLAB_utils. It does all the
	 * processing of the data and alters the state of the class to reflect the
	 * acceptable routes (in the "new" routes objects).
	 * 
	 * @param startTime
	 * @param horizon
	 * @param rebWeight
	 * @return the percentage of vehicles that need to be utilized.
	 * @throws FileNotFoundException
	 * @throws IOException
	 * @throws MatlabConnectionException
	 * @throws MatlabInvocationException
	 */
	public double callCPLEXOptimizer(double startTime)
			throws FileNotFoundException, IOException, MatlabConnectionException, MatlabInvocationException {
		// Call MATLAB script to generate trips

		System.out.println("Starting optmizer... this could take a while");

		Collection<Vehicle> vehicles = optimContext.taxiData.getVehicles().values();
		double[] totalvehicles = new double[] { vehicles.size() };

		double[] distribution = calculateVehicleDistribution(vehicles);
		double[] totalpassengers = new double[] { stats.numberOfPassengers() };
		int numLinks = optimContext.network.getLinks().values().size();
		double[] vehiclelocations = AMoDSchedulingUtils.calculateVehicleLocations(vehicles, numLinks, startTime);
		double[] waitingpassengers = AMoDSchedulingUtils.calculateNumberWaitingPassengers(rolloverRequests,
				nodes_to_stations, stations_to_nodes);

		MLDouble matdistribution = new MLDouble("vehdistribution", distribution, distribution.length);
		MLDouble mattotalvehicles = new MLDouble("totalvehicles", totalvehicles, 1);
		MLDouble mattotalpassengers = new MLDouble("totalpassengers", totalpassengers, 1);
		MLDouble matlocations = new MLDouble("vehlocations", vehiclelocations, vehiclelocations.length);
		MLDouble matwaitingpass = new MLDouble("waitingpassengers", waitingpassengers, waitingpassengers.length);

		ArrayList<MLArray> list = new ArrayList<MLArray>();
		list.add(matdistribution);
		list.add(mattotalvehicles);
		list.add(mattotalpassengers);
		list.add(matlocations);
		list.add(matwaitingpass);

		new MatFileWriter("MATLAB_utils/vehicledata.mat", list);

		proxy.eval("cd('~/SVN/code/AMoD_congestion/AMoD_taxi/MATLAB_utils')");
		String timeHorizonAsStr = Integer.toString(timeHorizon);
		String optimstartTime = Double.toString(startTime);
		String commandToRun = "runOptimization(" + optimstartTime + "," + timeHorizonAsStr + "," + rebWeight + ","
				+ tripThreshold + "," + (legacyRebalance ? 1 : 0) + ",'" + optimizerDataFile + "')";
		String displayCommand = "disp('runOptimization(" + optimstartTime + "," + timeHorizonAsStr + "," + rebWeight
				+ "," + tripThreshold + "," + (legacyRebalance ? 1 : 0) + "," + optimizerDataFile + ")')";
		proxy.eval(displayCommand);
		proxy.eval(commandToRun);

		double required_vehicles;
		if (!legacyRebalance) {
			MatFileReader matfilereader = new MatFileReader("src/main/resources/optimizerpaths.mat");
			MLCell passpaths = (MLCell) matfilereader.getMLArray("passpaths");
			MLCell rebpaths = (MLCell) matfilereader.getMLArray("rebpaths");
			MLDouble numvehiclesarr = (MLDouble) matfilereader.getMLArray("numvehicles");
			double[][] numvehiclesarrjava = numvehiclesarr.getArray();
			required_vehicles = numvehiclesarrjava[0][0];

			if (use_stations) {
				new_station_routes = new HashMap<Integer, Map<Integer, Set<Path>>>(); // used
																						// for
																						// station-wise
																						// routing
				new_station_reb_routes = new HashMap<Integer, List<Path>>();
				decomposePassPathsStations(new_station_routes, passpaths);
				decomposeRebPathsStations(new_station_reb_routes, rebpaths);

			} else {
				new_routes = new HashMap<Link, Set<Path>>();
				new_reb_routes = new HashMap<Link, Set<Path>>();

				decomposePassPaths(new_routes, passpaths);
				decomposePassPaths(new_reb_routes, rebpaths);
			}
		} else {
			MatFileReader matfilereader = new MatFileReader("src/main/resources/optimizerpairs.mat");
			MLCell matRebQueue = (MLCell) matfilereader.getMLArray("rebalanceQueue");
			MLDouble numvehiclesarr = (MLDouble) matfilereader.getMLArray("numvehicles");
			double[][] numvehiclesarrjava = numvehiclesarr.getArray();
			required_vehicles = numvehiclesarrjava[0][0];

			rebalance_queue = makeRebQueue(matRebQueue);

		}

		return required_vehicles / stats.numberIdle();
		// Modified below by Federico. We want to always rebalance. How bad
		// could it be? remember to change back.
		// return 1;
	}

	private List<List<Integer>> makeRebQueue(MLCell matRebQueue) {
		List<List<Integer>> result = new ArrayList<List<Integer>>();
		List<MLArray> stationPairs = matRebQueue.cells();
		for (int i = 0; i < stationPairs.size(); i++) {
			result.add(new ArrayList<Integer>());
			MLDouble curr = (MLDouble) stationPairs.get(i);
			double[][] arr = curr.getArray();
			for (int j = 0; j < arr.length; j++) {
				result.get(i).add((int) arr[j][0]);
			}
		}
		return result;
	}

	/**
	 * Function calculateVehicleDistribution
	 * ----------------------------------------- This function takes in a
	 * collection of vehicles and determines the distribution of them. The
	 * distribution is represented as an array where the ith entry represents
	 * how many vehicles are at the ith station.
	 * 
	 * @param vehicles
	 * @return the vehicle distribution
	 */
	private double[] calculateVehicleDistribution(Collection<Vehicle> vehicles) {
		double[] result = new double[stations_to_nodes.length];

		for (Vehicle v : vehicles) {
			Schedule<TaxiTask> currSched = TaxiSchedules.asTaxiSchedule(v.getSchedule());
			Link lastLink = Schedules.getLastLinkInSchedule(currSched);
			int node = Integer.parseInt(lastLink.getToNode().getId().toString());
			int station = (int) nodes_to_stations[node - 1][0];
			result[station - 1]++;
		}
		return result;
	}

	/**
	 * Function: initMatlab --------------------------------------------------
	 * This function gets the proxy object that will connect Java to MATLAB.
	 * 
	 * @throws MatlabConnectionException:
	 *             this occurs when the connection to MATLAB fails
	 * @throws IOException
	 * @throws FileNotFoundException
	 */
	private void initMatlab() throws MatlabConnectionException, FileNotFoundException, IOException {
		proxy = MatlabConnector.getProxy();
		MatFileReader matfilereader = new MatFileReader(stationMap);
		MLDouble station_to_node = (MLDouble) matfilereader.getMLArray("stationstonodes");
		stations_to_nodes = station_to_node.getArray();
		MLDouble node_to_station = (MLDouble) matfilereader.getMLArray("nodestostations");
		nodes_to_stations = node_to_station.getArray();

	}

	/**
	 * Function: notifyMobsimBeforeSimStep ------------------------------------
	 * This function allows the optimizer to get into the process of the
	 * controler and do what it needs to do. Every time a new decision epoch is
	 * reached, it first calls the cplex optimizer in MATLAB and then prints out
	 * some diagnostics. It also sets rebalancing routes.
	 * 
	 * @param MobsimBeforeSimStepEvent
	 *            e
	 */
	@Override
	public void notifyMobsimBeforeSimStep(@SuppressWarnings("rawtypes") MobsimBeforeSimStepEvent e) {
		simTime = (int) e.getSimulationTime();

		if (simTime >= optimizerEndTime + optimizerDelay) {
			if (use_stations) {

				station_routes = new_station_routes;
				station_reb_routes = new_station_reb_routes;
			} else {
				routes = new_routes;
				reb_routes = new_reb_routes;
			}
		}

		if (isNewDecisionEpoch(simTime) && requiresReoptimization) {
			try {
				fraction_to_rebalance = callCPLEXOptimizer(simTime);
				// number that are assigned to rebalance in this time-step, NOT
				// total number rebalancing.

				// double percentage =
				// ((double)stats.num_using_routes)/stats.total_route_count*100;

				if (doUnscheduleAwaitingRequests) {
					unscheduleAwaitingRequests();
				}

				// TODO (1) use a seperate variable to decide upon updating the
				// timeline??
				// TODO (2) update timeline only if the algo really wants to
				// reschedule in this time step,
				// perhaps by checking if there are any unplanned requests??
				if (doUnscheduleAwaitingRequests) {
					for (Vehicle v : optimContext.taxiData.getVehicles().values()) {
						optimContext.scheduler.updateTimeline(TaxiSchedules.asTaxiSchedule(v.getSchedule()));
					}
				}

				if (doUnscheduleAwaitingRequests && vehicleDiversion) {
					handleAimlessDriveTasks();
				}

				requiresReoptimization = true;
				System.out.println("Number of passengers in queue: " + unplannedRequests.size());
				printDiagnostics(simTime);
			} catch (IOException | MatlabConnectionException | MatlabInvocationException e1) {
				System.out.println("Oops, something went wrong!");
				e1.printStackTrace();
			}
		}

		int rebalancecount = rebalanceVehicles();
		scheduleUnplannedRequests();

	}

	private void printDiagnostics(double simTime) {
		System.out.println("Current Time: " + simTime);
		System.out.println("Total vehicles assigned to passenger trips: " + stats.total_route_count);
		System.out.println("Number of vehicles using our routes: " + stats.num_using_routes);
		// System.out.println("Percent of vehicles using MATLAB generated
		// routes: " + percentage + "%.");
		System.out.println("Number of passengers waiting: " + rolloverRequests.size());
		// System.out.println("Total vehicles set to rebalance at this timestep:
		// " + rebalancecount);
		System.out.println("Total number of vehicles serving: " + stats.numberServing());
		System.out.println("Total number of vehicles rebalancing: " + stats.numberRebalancing());
		System.out.println("Total number of vehicles chilling: " + stats.numberIdle());
	}

	private int rebalanceVehicles() {
		int rebalancecount;
		if (use_stations) {
			if (legacyRebalance) {
				rebalancecount = new AMoDStationSchedulingProblem(optimContext, dispatchFinder, router,
						nodes_to_stations, stations_to_nodes).legacyRebalanceVehicles(rebalance_queue,
								fraction_to_rebalance);
			} else {
				rebalancecount = new AMoDStationSchedulingProblem(optimContext, dispatchFinder, router,
						nodes_to_stations, stations_to_nodes).rebalanceVehicles(station_reb_routes,
								fraction_to_rebalance);
			}

		} else {
			rebalancecount = new AMoDNodesSchedulingProblem(optimContext, dispatchFinder, router)
					.rebalanceVehicles(reb_routes, fraction_to_rebalance);
		}
		return rebalancecount;
	}

	/**
	 * Function isNewDecisionEpoch ------------------------------------- This
	 * function takes in the time of the simulation and determines whether
	 * reoptimization is necessary.
	 * 
	 * @param double
	 *            simTime
	 * @return whether or not the reoptimization is necessary.
	 */
	protected boolean isNewDecisionEpoch(double simTime) {
		return simTime % reoptimizationTimeStep == 0;
	}

	/**
	 * Function scheduleUnplannedRequests
	 * -------------------------------------------------- This function
	 * schedules all of the unscheduled requests in the unplannedRequests queue.
	 * The AMoDSchedulingProblem class handles the nitty-gritty details of this.
	 * 
	 * @param simTime
	 */
	protected void scheduleUnplannedRequests() {
		if ((int) (simTime) % reschedulePeriod == 0) {
			unplannedRequests.addAll(rolloverRequests);
			rolloverRequests.clear();
		}
		int[] tempstats = new int[2];
		if (use_stations) {
			try {
				Set<TaxiRequest> newRollovers = new AMoDStationSchedulingProblem(optimContext, dispatchFinder, router,
						nodes_to_stations, stations_to_nodes)
								.scheduleUnplannedRequests((Queue<TaxiRequest>) unplannedRequests, station_routes);
				rolloverRequests.addAll(newRollovers);
			} catch (FileNotFoundException | UnsupportedEncodingException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		} else {
			tempstats = new AMoDNodesSchedulingProblem(optimContext, dispatchFinder, router)
					.scheduleUnplannedRequests((List<TaxiRequest>) unplannedRequests, routes, reb_routes);
		}

		// stats.num_using_routes += tempstats[0];
		// stats.total_route_count += tempstats[1];
		// stats.vehiclesServing = tempstats[1];
	}

	/**
	 * Function nextTask ------------------------------------- This function
	 * takes in a schedule, shifts its timings to match the current state of the
	 * system, and then advances to the next step.
	 * 
	 * @param Schedule<?
	 *            extends Task> schedule
	 */
	@Override
	public void nextTask(Schedule<? extends Task> schedule) {
		shiftTimings(schedule);
		schedule.nextTask();
	}

	/**
	 * Function shiftTimings ------------------------------------- This function
	 * takes in a schedule and shifts the timing of all the events such that the
	 * next task in the schedule starts right now. This is so that when moving
	 * to the next task of a schedule before the current task has ended, the
	 * following tasks are performed right away.
	 * 
	 * @param schedule
	 */
	private void shiftTimings(Schedule<? extends Task> schedule) {
		Schedule<TaxiTask> taxischedule = TaxiSchedules.asTaxiSchedule(schedule);
		if (taxischedule.getStatus() != ScheduleStatus.STARTED) {
			return;
		}

		double now = optimContext.timer.getTimeOfDay();
		Task currentTask = schedule.getCurrentTask();
		double diff = now - currentTask.getEndTime();

		if (diff == 0) {
			return;
		}

		currentTask.setEndTime(now);

		List<TaxiTask> tasks = taxischedule.getTasks();
		int nextTaskIdx = currentTask.getTaskIdx() + 1;

		// all except the last task (waiting)
		for (int i = nextTaskIdx; i < tasks.size() - 1; i++) {
			Task task = tasks.get(i);
			task.setBeginTime(task.getBeginTime() + diff);
			task.setEndTime(task.getEndTime() + diff);
		}

		// wait task
		if (nextTaskIdx != tasks.size()) {
			Task waitTask = tasks.get(tasks.size() - 1);
			waitTask.setBeginTime(waitTask.getBeginTime() + diff);

			double tEnd = Math.max(waitTask.getBeginTime(), taxischedule.getVehicle().getT1());
			waitTask.setEndTime(tEnd);
		}
	}

	/**
	 * Function: decomposePassPaths ------------------------------------- This
	 * function takes in the collection that holds all the routes and one single
	 * MLCell which represents one route and adds the parsed path into the
	 * above-mentioned collection.
	 * 
	 * @param routes
	 * @param j
	 */
	private void decomposePassPaths(Map<Link, Set<Path>> routes, MLCell j) {
		Network network = optimContext.network;
		Map<Id<Node>, ? extends Node> nodenetwork = network.getNodes();
		ArrayList<MLArray> paths = j.cells();
		for (MLArray path : paths) {
			MLCell cellpath = (MLCell) path;
			if (path.getM() != 0) {
				List<Node> nodelist = new ArrayList<Node>();
				List<Link> linklist = new ArrayList<Link>();
				MLDouble route = (MLDouble) (cellpath.get(0));
				double[][] nodes = route.getArray();
				for (int i = 0; i < nodes.length - 1; i++) {
					int currnode = (int) nodes[i][0];
					int nextnode = (int) nodes[i + 1][0];
					Node start = nodenetwork.get(Id.createNodeId(Integer.toString(currnode)));
					Node finish = nodenetwork.get(Id.createNodeId(Integer.toString(nextnode)));
					nodelist.add(start);
					Collection<? extends Link> possiblelinks = start.getOutLinks().values();
					for (Link l : possiblelinks) {
						if (l.getToNode().equals(finish)) {
							linklist.add(l);
							break;
						}
					}
					if (i == nodes.length - 2) {
						nodelist.add(finish);
					}
				}
				Path temppath = AMoDSchedulingUtils.convertNodeListtoPath(nodelist, linklist);
				if (routes.get(linklist.get(0)) == null) { // add path to
															// routelist
					routes.put(linklist.get(0), new HashSet<Path>());
				}
				routes.get(linklist.get(0)).add(temppath);
			}
		}
	}

	/**
	 * Function: decomposePassPathsStations ------------------------------------
	 * This function takes in an MLCell that contains route information and adds
	 * it to the map of station-routes that is used by the optimizer. It does so
	 * by breaking it down into its component parts and storing it as a
	 * LeastCostPathCalculator.Path. It is then stored in the map by its
	 * stations. The start station is the first key, and the end station is the
	 * second key.
	 * 
	 * @param new_station_routes
	 * @param j
	 */
	private void decomposePassPathsStations(Map<Integer, Map<Integer, Set<Path>>> new_station_routes, MLCell j) {
		Network network = optimContext.network;
		Map<Id<Node>, ? extends Node> nodenetwork = network.getNodes();
		if (j == null) {
			return;
		}
		ArrayList<MLArray> paths = j.cells();
		for (MLArray path : paths) {
			MLCell cellpath = (MLCell) path;
			if (path.getM() != 0) {
				List<Node> nodelist = new ArrayList<Node>();
				List<Link> linklist = new ArrayList<Link>();
				MLDouble route = (MLDouble) (cellpath.get(0));
				double[][] nodes = route.getArray();
				for (int i = 0; i < nodes.length - 1; i++) {
					int currnode = (int) nodes[i][0];
					int nextnode = (int) nodes[i + 1][0];
					Node start = nodenetwork.get(Id.createNodeId(Integer.toString(currnode)));
					Node finish = nodenetwork.get(Id.createNodeId(Integer.toString(nextnode)));
					nodelist.add(start);
					Collection<? extends Link> possiblelinks = start.getOutLinks().values();
					for (Link l : possiblelinks) {
						if (l.getToNode().equals(finish)) {
							linklist.add(l);
							break;
						}
					}
					if (i == nodes.length - 2) {
						nodelist.add(finish);
					}
				}
				Path temppath = AMoDSchedulingUtils.convertNodeListtoPath(nodelist, linklist);
				String id_of_startnode = nodelist.get(0).getId().toString();
				int start_station = (int) nodes_to_stations[Integer.parseInt(id_of_startnode)][0];
				String id_of_endnode = nodelist.get(nodelist.size() - 1).getId().toString();
				int end_station = (int) nodes_to_stations[Integer.parseInt(id_of_endnode)][0];
				if (new_station_routes.get(start_station) == null) {
					new_station_routes.put(start_station, new HashMap<Integer, Set<Path>>());
				}
				if (new_station_routes.get(start_station).get(end_station) == null) {
					new_station_routes.get(start_station).put(end_station, new HashSet<Path>());
				}
				new_station_routes.get(start_station).get(end_station).add(temppath);
			}
		}
	}

	/**
	 * Function: decomposeRebPathsStations ----------------------------------
	 * This function has similar functionality to the decomposePassPaths
	 * function except for how it stores the data. It stores the data as a Map
	 * where the key is a station and the value is a set of possible rebalancing
	 * routes that a car at that station could take.
	 * 
	 * @param new_station_reb_routes2
	 * @param rebpaths
	 */
	private void decomposeRebPathsStations(Map<Integer, List<Path>> new_station_reb_routes2, MLCell rebpaths) {
		Network network = optimContext.network;
		Map<Id<Node>, ? extends Node> nodenetwork = network.getNodes();
		ArrayList<MLArray> paths = rebpaths.cells();
		for (MLArray path : paths) {
			MLCell cellpath = (MLCell) path;
			if (path.getM() != 0) {
				List<Node> nodelist = new ArrayList<Node>();
				List<Link> linklist = new ArrayList<Link>();
				MLDouble route = (MLDouble) (cellpath.get(0));
				double[][] nodes = route.getArray();
				for (int i = 0; i < nodes.length - 1; i++) {
					int currnode = (int) nodes[i][0];
					int nextnode = (int) nodes[i + 1][0];
					Node start = nodenetwork.get(Id.createNodeId(Integer.toString(currnode)));
					Node finish = nodenetwork.get(Id.createNodeId(Integer.toString(nextnode)));
					nodelist.add(start);
					Collection<? extends Link> possiblelinks = start.getOutLinks().values();
					for (Link l : possiblelinks) {
						if (l.getToNode().equals(finish)) {
							linklist.add(l);
							break;
						}
					}
					if (i == nodes.length - 2) {
						nodelist.add(finish);
					}
				}
				Path temppath = AMoDSchedulingUtils.convertNodeListtoPath(nodelist, linklist);
				if (nodelist.size() > 0) {
					String id_of_startnode = nodelist.get(0).getId().toString();
					int start_station = (int) nodes_to_stations[Integer.parseInt(id_of_startnode) - 1][0];
					if (new_station_reb_routes2.get(start_station) == null) {
						new_station_reb_routes2.put(start_station, new ArrayList<Path>());
					}
					new_station_reb_routes2.get(start_station).add(temppath);
				}
			}
		}
	}
}
