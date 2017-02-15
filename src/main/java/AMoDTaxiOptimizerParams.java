import org.apache.commons.configuration.Configuration;
import org.matsim.contrib.taxi.optimizer.AbstractTaxiOptimizerParams;

/**
 * Class: AMoDTaxiOptimizerParams
 * 
 * This class contains the parameters necessary to run the AMoDOptimizer 
 * 
 * @author yhindy
 *
 */

public class AMoDTaxiOptimizerParams 
	extends AbstractTaxiOptimizerParams
{
	 /** The string identifiers of the parameters for the XML file */
	 private final String OPTIMIZER_DELAY = "optimizerDelay";
	 private final String REBALANCE_WEIGHT = "rebalanceWeight";
	 private final String OPTIMIZER_DATA = "optimizerDataFile";
	 private final String TIME_HORIZON = "timeHorizon";
	 private final String TRIP_THRESHOLD = "tripThreshold";
	 private final String END_TIME = "endTime";
	 private final String VEHICLE_DIVERSION = "vehicleDiversion";
	 private final String STATION_MAP = "stationMap";
	 private final String LEGACY_REBALANCE = "legacyRebalance";
	 private final String NEIGHBOURHOOD_SIZE = "neighbourhoodSize";
	 private final String RESCHEDULE_PERIOD = "reschedulePeriod";
	 private final String AMOD_DISPATCH = "amodDispatch";
	 
	 /** The actual parameters */
	 public int optimizerDelay;
	 public double rebWeight;
	 public String optimizerDataFile;
	 public int timeHorizon;
	 public double tripThreshold;
	 public int endTime;
	 public String stationMap;
	 public boolean legacyRebalance;
	 public int neighbourhoodSize;
	 public int reschedulePeriod;
	 public boolean amodDispatch;

	 
	 /** 
	  * Constructor AMoDTaxiOptimizerParams
	  * ----------------------------------
	  * This constructor takes in the optimizerConfig group and 
	  * extracts the necessary and important parameters for 
	  * running the simulation. 
	  * 
	  * @param optimizerConfig
	  */
	 public AMoDTaxiOptimizerParams(Configuration optimizerConfig)
	    {
		    super(optimizerConfig);
			optimizerDelay = optimizerConfig.getInt(OPTIMIZER_DELAY);
			rebWeight = optimizerConfig.getDouble(REBALANCE_WEIGHT);
			optimizerDataFile = optimizerConfig.getString(OPTIMIZER_DATA);
			timeHorizon = optimizerConfig.getInt(TIME_HORIZON);
			tripThreshold = optimizerConfig.getDouble(TRIP_THRESHOLD);
			stationMap = optimizerConfig.getString(STATION_MAP);
			legacyRebalance = optimizerConfig.getBoolean(LEGACY_REBALANCE);
			neighbourhoodSize = optimizerConfig.getInt(NEIGHBOURHOOD_SIZE);
			reschedulePeriod = optimizerConfig.getInt(RESCHEDULE_PERIOD);
			amodDispatch = optimizerConfig.getBoolean(AMOD_DISPATCH);
			
			//endTime = optimizerConfig.getInt(END_TIME);
	    }
}
