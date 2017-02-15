import org.matsim.contrib.dvrp.path.VrpPathWithTravelData;
import org.matsim.contrib.taxi.schedule.TaxiEmptyDriveTask;

/**
 * Class: AMoDRebalanceTask
 * 
 * This class represents a rebalancing drive task. It is
 * distinct from the TaxiEmptyDriveTask for data gathering purposes.
 * @author yhindy
 *
 */
public class AMoDRebalanceTask
	extends TaxiEmptyDriveTask
{
	/**
	 * Constructor: AMoDRebalanceTask
	 * ------------------------------
	 * Creates an AMoDRebalanceTask based on the given path. 
	 * 
	 * @param path
	 */
	public AMoDRebalanceTask(VrpPathWithTravelData path) {
		super(path);
	}
}
