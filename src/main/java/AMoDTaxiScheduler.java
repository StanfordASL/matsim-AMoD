import org.matsim.api.core.v01.Scenario;
import org.matsim.contrib.dvrp.data.Vehicle;
import org.matsim.contrib.dvrp.path.VrpPathWithTravelData;
import org.matsim.contrib.dvrp.schedule.Schedule;
import org.matsim.contrib.dvrp.schedule.Schedules;
import org.matsim.contrib.taxi.data.TaxiData;
import org.matsim.contrib.taxi.data.TaxiRequest;
import org.matsim.contrib.taxi.data.TaxiRequest.TaxiRequestStatus;
import org.matsim.contrib.taxi.schedule.TaxiDropoffTask;
import org.matsim.contrib.taxi.schedule.TaxiOccupiedDriveTask;
import org.matsim.contrib.taxi.schedule.TaxiPickupTask;
import org.matsim.contrib.taxi.schedule.TaxiSchedules;
import org.matsim.contrib.taxi.schedule.TaxiTask;
import org.matsim.contrib.taxi.scheduler.TaxiScheduler;
import org.matsim.contrib.taxi.scheduler.TaxiSchedulerParams;
import org.matsim.core.mobsim.framework.MobsimTimer;
import org.matsim.core.router.util.TravelDisutility;
import org.matsim.core.router.util.TravelTime;

public class AMoDTaxiScheduler
				extends TaxiScheduler{

	public AMoDTaxiScheduler(Scenario scenario, TaxiData taxiData, MobsimTimer timer, TaxiSchedulerParams params,
			TravelTime travelTime, TravelDisutility travelDisutility) {
		super(scenario, taxiData, timer, params, travelTime, travelDisutility);
	}
	
	public void scheduleFirstHalf(Vehicle vehicle, TaxiRequest request, VrpPathWithTravelData toPickup) 
	{
		if (request.getStatus() != TaxiRequestStatus.UNPLANNED) {
			throw new IllegalStateException();
		}
		
		Schedule<TaxiTask> schedule = TaxiSchedules.asTaxiSchedule(vehicle.getSchedule());
		divertOrAppendDrive(schedule, toPickup);
		
		double pickupEndTime = Math.max(toPickup.getArrivalTime(), request.getT0())
				+ params.pickupDuration;
		schedule.addTask(new TaxiPickupTask(toPickup.getArrivalTime(), pickupEndTime, request));
	}

	public double calcJourneyStartTime(Schedule<TaxiTask> schedule) {
		TaxiPickupTask pickupStayTask = (TaxiPickupTask)Schedules.getLastTask(schedule);
		return pickupStayTask.getEndTime();
	}

	public void scheduleSecondHalf(Vehicle v, TaxiRequest req, VrpPathWithTravelData p2) {
		Schedule<TaxiTask> schedule = TaxiSchedules.asTaxiSchedule(v.getSchedule());
		
		schedule.addTask(new TaxiOccupiedDriveTask(p2, req));
		double t4 = p2.getArrivalTime();
		double t5 = t4 + params.dropoffDuration;
		schedule.addTask(new TaxiDropoffTask(t4, t5, req));
		
		appendStayTask(schedule);
	}
}
