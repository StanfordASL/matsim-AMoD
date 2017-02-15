import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Collection;

import org.apache.commons.configuration.Configuration;
import org.apache.commons.configuration.MapConfiguration;
import org.matsim.api.core.v01.Scenario;
import org.matsim.contrib.dvrp.passenger.PassengerEngine;
import org.matsim.contrib.dvrp.router.TimeAsTravelDisutility;
import org.matsim.contrib.dvrp.trafficmonitoring.VrpTravelTimeModules;
import org.matsim.contrib.dvrp.vrpagent.*;
import org.matsim.contrib.dvrp.vrpagent.VrpLegs.LegCreator;
import org.matsim.contrib.taxi.data.TaxiData;
import org.matsim.contrib.taxi.optimizer.*;
import org.matsim.contrib.taxi.passenger.TaxiRequestCreator;
import org.matsim.contrib.taxi.run.TaxiConfigGroup;
import org.matsim.contrib.taxi.run.TaxiModule;
import org.matsim.contrib.taxi.scheduler.*;
import org.matsim.contrib.taxi.vrpagent.TaxiActionCreator;
import org.matsim.core.api.experimental.events.EventsManager;
import org.matsim.core.config.ConfigGroup;
import org.matsim.core.mobsim.framework.Mobsim;
import org.matsim.core.mobsim.qsim.*;
import org.matsim.core.router.util.*;
import org.matsim.vehicles.VehicleType;

import com.google.inject.*;
import com.google.inject.name.Named;

import matlabcontrol.MatlabConnectionException;

/**
 * Class: AMoDQSimProvider
 * 
 * This class provides the functionality for having the AMoDOptimizer
 * be a part of the simulation. 
 * 
 * @author yhindy
 *
 */
public class AMoDQSimProvider
    implements Provider<Mobsim>
{
	/*  ########################
	 *  ## Instance variables ##
	 *  ########################
	 */
	
	
    private final EventsManager eventsManager;
    private final Collection<AbstractQSimPlugin> plugins;

    protected final Scenario scenario;
    protected final TaxiData taxiData;
    protected final TravelTime travelTime;

    protected final TaxiConfigGroup taxiCfg;
    private final VehicleType vehicleType;


    /**
     * Constructor: AMoDQSimProvider
     * ------------------------------
     * This constructor takes in all the necessary data for the AMoDOptimizer and QSim 
     * and stores them in the instance variables. 
     * 
     * @param eventsManager
     * @param plugins
     * @param scenario
     * @param taxiData
     * @param travelTime
     * @param vehicleType
     * @param optimizerFactory
     */
    @Inject
    public AMoDQSimProvider(EventsManager eventsManager, Collection<AbstractQSimPlugin> plugins,
            Scenario scenario, TaxiData taxiData,
            @Named(VrpTravelTimeModules.DVRP_ESTIMATED) TravelTime travelTime,
            @Named(TaxiModule.TAXI_MODE) VehicleType vehicleType,
            TaxiOptimizerFactory optimizerFactory)
    {
        this.eventsManager = eventsManager;
        this.plugins = plugins;
        this.scenario = scenario;
        this.taxiData = taxiData;
        this.travelTime = travelTime;
        this.taxiCfg = TaxiConfigGroup.get(scenario.getConfig());
        this.vehicleType = vehicleType;
    }


    /** 
     * Function: get
     * --------------------------
     * This function creates a QSim and adds all of the necessary plugins into it
     * so that it can run the dvrp module and the taxi module. 
     * 
     * @return the packaged QSim.
     */
    @Override
    public Mobsim get()
    {
        if (taxiCfg.isVehicleDiversion() && !taxiCfg.isOnlineVehicleTracker()) {
            throw new IllegalStateException("Diversion requires online tracking");
        }

        QSim qSim = QSimUtils.createQSim(scenario, eventsManager, plugins);

        TaxiOptimizer optimizer = null;
		try {
			optimizer = createTaxiOptimizer(qSim);
		} catch (MatlabConnectionException | IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        qSim.addQueueSimulationListeners(optimizer);

        PassengerEngine passengerEngine = createPassengerEngine(optimizer);
        qSim.addMobsimEngine(passengerEngine);
        qSim.addDepartureHandler(passengerEngine);

        VrpAgentSource agentSource = createVrpAgentSource(optimizer, qSim, passengerEngine,
                vehicleType);
        qSim.addAgentSource(agentSource);

        return qSim;
    }


    /**
     * Function createTaxiOptimizer
     * --------------------------------
     * This function takes in a QSim and creates the AMoDTaxiOptimizer. 
     * 
     * @param qSim
     * @return the finished AMoDTaxiOptimizer
     * @throws MatlabConnectionException
     * @throws IOException 
     * @throws FileNotFoundException 
     */
    protected TaxiOptimizer createTaxiOptimizer(QSim qSim) throws MatlabConnectionException, FileNotFoundException, IOException
    {
        TaxiSchedulerParams schedulerParams = new TaxiSchedulerParams(taxiCfg);
        TravelDisutility travelDisutility = new TimeAsTravelDisutility(travelTime);
        AMoDTaxiScheduler scheduler = new AMoDTaxiScheduler(scenario, taxiData, qSim.getSimTimer(),
                schedulerParams, travelTime, travelDisutility);

        TaxiOptimizerContext optimContext = new TaxiOptimizerContext(taxiData,
                scenario.getNetwork(), qSim.getSimTimer(), travelTime, travelDisutility, scheduler);
        
        ConfigGroup optimizerConfigGroup = taxiCfg.getOptimizerConfigGroup();
        Configuration optimizerConfig = new MapConfiguration(optimizerConfigGroup.getParams());
        
        return new AMoDTaxiOptimizer(optimContext, new AMoDTaxiOptimizerParams(optimizerConfig));
    }

    /** 
     * Function createPassengerEngine
     * --------------------------------
     * This function creates a new passenger engine for the Mobsim to run.
     * 
     * @param optimizer
     * @return the finished PassengerEngine
     */
    protected PassengerEngine createPassengerEngine(TaxiOptimizer optimizer)
    {
        return new PassengerEngine(TaxiModule.TAXI_MODE, eventsManager, new TaxiRequestCreator(),
                optimizer, taxiData, scenario.getNetwork());
    }

    /**
     * 
     * Function createVrpAgentSource
     * ---------------------------------
     * This function creates the agent sources for the Mobsim 
     * 
     * @param optimizer
     * @param qSim
     * @param passengerEngine
     * @param vehicleType
     * @return the finished agent source.
     */
    protected VrpAgentSource createVrpAgentSource(TaxiOptimizer optimizer, QSim qSim,
            PassengerEngine passengerEngine, VehicleType vehicleType)
    {
        LegCreator legCreator = taxiCfg.isOnlineVehicleTracker() ? //
                VrpLegs.createLegWithOnlineTrackerCreator(optimizer, qSim.getSimTimer()) : //
                VrpLegs.createLegWithOfflineTrackerCreator(qSim.getSimTimer());
        TaxiActionCreator actionCreator = new TaxiActionCreator(passengerEngine, legCreator,
                taxiCfg.getPickupDuration());
        return new VrpAgentSource(actionCreator, taxiData, optimizer, qSim, vehicleType);
    }
}