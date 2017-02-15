import org.matsim.contrib.taxi.optimizer.TaxiOptimizerContext;
import org.matsim.contrib.taxi.schedule.TaxiDropoffTask;
import org.matsim.contrib.taxi.schedule.TaxiEmptyDriveTask;
import org.matsim.contrib.taxi.schedule.TaxiOccupiedDriveTask;
import org.matsim.contrib.taxi.schedule.TaxiPickupTask;
import org.matsim.contrib.taxi.schedule.TaxiSchedules;
import org.matsim.contrib.taxi.schedule.TaxiStayTask;
import org.matsim.contrib.taxi.schedule.TaxiTask;
import org.matsim.contrib.dvrp.data.Vehicle;
import org.matsim.contrib.dvrp.schedule.Schedule;
import org.matsim.contrib.dvrp.schedule.Schedule.ScheduleStatus;

import java.util.Collection;

/**
 * Class: AMoDPerformance
 * 
 * This class contains the functionality required to keep track
 * of certain statistics of the simulation. They are very basic 
 * and do not represent all of the analysis that can be done
 * using events.
 * 
 * @author yhindy
 */
public class AMoDPerformance {
	
	/*  ########################
	 *  ## Instance variables ##
	 *  ########################
	 */
	
	/** The number of vehicles using cplex routes. */
	public double num_using_routes;
	/** The total number of vehicles that were routed. */
	public double total_route_count;
	
	/* TODO: what are these? 
	 * public double tripThreshold;
	 * public int vehiclesRebalancing; */
	
	public int vehiclesServing;
	
	/** All of the vehicles in the simulation */
	private Collection<Vehicle> vehicles;
	
	/** The number of vehicles in the simulation */
	public int totalVehicles;
	
	public TaxiOptimizerContext optimcontext;
	
	/**
	 * Constructor: AMoDPerformance
	 * -----------------------------
	 * This constructor initializes the AMoDPerformance object and gives
	 * it the necessary data it needs to compute various statistics.
	 * 
	 * @param optimcontext
	 */
	public AMoDPerformance(TaxiOptimizerContext optimcontext)
	{
		vehicles = optimcontext.taxiData.getVehicles().values();
		totalVehicles = vehicles.size();
		this.optimcontext = optimcontext;
	}
	
	/**
	 * Function numberRebalancing
	 * -------------------------
	 * This function counts how many vehicles are in the process of rebalancing. 
	 * @return the number of rebalancing vehicles
	 */
	public int numberRebalancing() {
		int count = 0;
		for (Vehicle v : vehicles) {
			Schedule<TaxiTask> curr = TaxiSchedules.asTaxiSchedule(v.getSchedule());
			if (curr.getStatus() == ScheduleStatus.STARTED) {
				TaxiTask currtask = curr.getCurrentTask();
				if (currtask instanceof AMoDRebalanceTask) {
					count++;
				}
			}
			
		}
		
		return count;
	}
	
	/**
	 * Function numberServing
	 * ----------------------
	 * This function counts how many vehicles are currently serving customers.
	 * @return number of busy cars
	 */
	public int numberServing() {
		int count = 0;
		for (Vehicle v : vehicles) {
			Schedule<TaxiTask> curr = TaxiSchedules.asTaxiSchedule(v.getSchedule());
			if (curr.getStatus() == ScheduleStatus.STARTED) {
				TaxiTask currtask = curr.getCurrentTask();
				if (currtask instanceof TaxiOccupiedDriveTask
						|| currtask instanceof TaxiEmptyDriveTask
						&& !(currtask instanceof AMoDRebalanceTask)) {
					count++;
				}
			}
		}
		
		return count;
	}
	
	/**
	 * Function: numberOfPassengers
	 * ------------------------------
	 * This returns the number of passengers currently being served
	 * in the system. It is a bit buggy because it does not include
	 * passengers who are waiting, but a fix for that will come later.
	 * @return the number of passengers
	 */
	public int numberOfPassengers() {
		int count = 0;
		for (Vehicle v : vehicles) {
			Schedule<TaxiTask> curr = TaxiSchedules.asTaxiSchedule(v.getSchedule());
			if (curr.getStatus() == ScheduleStatus.STARTED) {
				TaxiTask currtask = curr.getCurrentTask();
				if (currtask instanceof TaxiOccupiedDriveTask
						|| currtask instanceof TaxiEmptyDriveTask
						|| currtask instanceof TaxiPickupTask
						|| currtask instanceof TaxiDropoffTask
						&& !(currtask instanceof AMoDRebalanceTask)) {
					count++;
				}
			}
		}
		return count;
	}
	
	/**
	 * Function numberIdle
	 * --------------------
	 * This function computes the number of vehicles that aren't doing anything,
	 * i.e. waiting. 
	 * @return the number of idle vehicles.
	 */
	public int numberIdle() {
		int count = 0;
		for (Vehicle v : vehicles) {
			if (optimcontext.scheduler.isIdle(v)) {
				count++;
			}
		}
		return count;
	}
}
