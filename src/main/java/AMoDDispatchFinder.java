import java.util.HashMap;
import java.util.Map;

import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.contrib.dvrp.data.Vehicle;
import org.matsim.contrib.dvrp.path.VrpPathWithTravelData;
import org.matsim.contrib.dvrp.path.VrpPaths;
import org.matsim.contrib.dvrp.util.LinkTimePair;
import org.matsim.api.core.v01.network.Node;
import org.matsim.contrib.taxi.data.TaxiRequest;
import org.matsim.contrib.taxi.optimizer.BestDispatchFinder;
import org.matsim.contrib.taxi.optimizer.LinkProvider;
import org.matsim.contrib.taxi.optimizer.LinkProviders;
import org.matsim.contrib.taxi.optimizer.TaxiOptimizerContext;
import org.matsim.contrib.taxi.optimizer.BestDispatchFinder.Dispatch;
import org.matsim.contrib.taxi.scheduler.TaxiScheduleInquiry;
import org.matsim.core.router.ArrayFastRouterDelegateFactory;
import org.matsim.core.router.FastMultiNodeDijkstra;
import org.matsim.core.router.FastRouterDelegateFactory;
import org.matsim.core.router.ImaginaryNode;
import org.matsim.core.router.InitialNode;
import org.matsim.core.router.MultiNodeDijkstra;
import org.matsim.core.router.util.ArrayRoutingNetworkFactory;
import org.matsim.core.router.util.PreProcessDijkstra;
import org.matsim.core.router.util.RoutingNetwork;
import org.matsim.core.router.util.LeastCostPathCalculator.Path;

public class AMoDDispatchFinder extends BestDispatchFinder {
	
	double[][] stations_to_nodes;
	double[][] nodes_to_stations;
	private final TaxiOptimizerContext optimContext;
	private int expectedNeighbourhoodSize;
	private final TaxiScheduleInquiry scheduleInquiry;
	private final MultiNodeDijkstra router;

	public AMoDDispatchFinder(TaxiOptimizerContext optimContext, int expectedNeighbourhoodSize, double[][] stations_to_nodes,
								double[][] nodes_to_stations) {
		super(optimContext, expectedNeighbourhoodSize);
		this.stations_to_nodes = stations_to_nodes;
		this.nodes_to_stations = nodes_to_stations;
		this.optimContext = optimContext;
		this.scheduleInquiry = optimContext.scheduler;
		this.expectedNeighbourhoodSize = expectedNeighbourhoodSize;
		
		PreProcessDijkstra preProcessDijkstra = null;
		FastRouterDelegateFactory fastRouterFactory = new ArrayFastRouterDelegateFactory();

		RoutingNetwork routingNetwork = new ArrayRoutingNetworkFactory(preProcessDijkstra)
				.createRoutingNetwork(optimContext.network);
		router = new FastMultiNodeDijkstra(routingNetwork, optimContext.travelDisutility, optimContext.travelTime,
				preProcessDijkstra, fastRouterFactory, false);
	}
	
	public Dispatch<TaxiRequest> findBestVehicleForRequest(TaxiRequest req,
			Iterable<? extends Vehicle> vehicles) {
		return findBestVehicle(req, vehicles, req.getFromLink());
	}
	
	public <D> Dispatch<D> findBestVehicle(D destination, Iterable<? extends Vehicle> vehicles,
			Link toLink) {
		if (vehicles == null) {
			System.out.println("Problem");
		}
		double currTime = optimContext.timer.getTimeOfDay();
		Node toNode = toLink.getFromNode();
		
		int station = (int) nodes_to_stations[Integer.parseInt(toNode.getId().toString())-1][0];
		
		Map<Id<Node>, Vehicle> nodeToVehicle = new HashMap<>(expectedNeighbourhoodSize);
		Map<Id<Node>, InitialNode> initialNodes = new HashMap<>(expectedNeighbourhoodSize);
		
		for (Vehicle veh : vehicles) {
			LinkTimePair departure = scheduleInquiry.getImmediateDiversionOrEarliestIdleness(veh);
			Link vehlink = departure.link;
			Id<Node> vehNodeId = vehlink.getToNode().getId();
			int vehStation = (int) nodes_to_stations[Integer.parseInt(vehNodeId.toString())-1][0];
			if (departure != null && station == vehStation) {
				Node vehNode;
                double delay = departure.time - currTime;
                if (departure.link == toLink) {
                    //hack: we are basically there (on the same link), so let's pretend vehNode == toNode
                    vehNode = toNode;
                }
                else {
                    vehNode = departure.link.getToNode();

                    //simplified, but works for taxis, since pickup trips are short (about 5 mins)
                    delay += 1 + toLink.getFreespeed(departure.time);
                }

                InitialNode existingInitialNode = initialNodes.get(vehNode.getId());
                if (existingInitialNode == null || existingInitialNode.initialCost > delay) {
                    InitialNode newInitialNode = new InitialNode(vehNode, delay, delay);
                    initialNodes.put(vehNode.getId(), newInitialNode);
                    nodeToVehicle.put(vehNode.getId(), veh);
                }
			}
		}
		
		if (initialNodes.isEmpty()) {
			return null;
		}
		
		ImaginaryNode fromNodes = router.createImaginaryNode(initialNodes.values());

        Path path = router.calcLeastCostPath(fromNodes, toNode, currTime, null, null);
        //the calculated path contains real nodes (no imaginary/initial nodes),
        //the time and cost are of real travel (between the first and last real node)
        //(no initial times/costs for imaginary<->initial are included)
        Node fromNode = path.nodes.get(0);
        Vehicle bestVehicle = nodeToVehicle.get(fromNode.getId());
        LinkTimePair bestDeparture = scheduleInquiry
                .getImmediateDiversionOrEarliestIdleness(bestVehicle);

        VrpPathWithTravelData vrpPath = VrpPaths.createPath(bestDeparture.link, toLink,
                bestDeparture.time, path, optimContext.travelTime);
        return new Dispatch<>(bestVehicle, destination, vrpPath);
		
		
		
	}

}
