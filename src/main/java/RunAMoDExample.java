import java.net.MalformedURLException;
import java.net.URL;

import org.matsim.api.core.v01.Scenario;
import org.matsim.contrib.dvrp.data.file.VehicleReader;
import org.matsim.contrib.dvrp.run.VrpQSimConfigConsistencyChecker;
import org.matsim.contrib.dvrp.trafficmonitoring.VrpTravelTimeModules;
import org.matsim.contrib.dynagent.run.DynQSimModule;
import org.matsim.contrib.otfvis.OTFVisLiveModule;
import org.matsim.contrib.taxi.data.TaxiData;
import org.matsim.contrib.taxi.run.TaxiConfigGroup;
import org.matsim.contrib.taxi.run.TaxiModule;
import org.matsim.core.config.*;
import org.matsim.core.controler.Controler;
import org.matsim.core.scenario.ScenarioUtils;
import org.matsim.vis.otfvis.OTFVisConfigGroup;

/**
 * Class: RunAMoDExample
 * 
 * This class actually runs the MATSim simulation using the AMoDTaxiOptimizer. 
 * 
 * @author yhindy
 *
 */
public class RunAMoDExample 
{
	/**
	 * Function: run
	 * ----------------
	 * This function runs the simulation based on the given configuration. 
	 * 
	 * @param configFile: string to config file
	 * @param otfvis: whether or not to visualize using OTFVIS
	 */
	public static void run(String configFile, boolean otfvis, boolean useTaxi)
    {
		Config config;
		if (useTaxi) {
			config = ConfigUtils.loadConfig(configFile, new TaxiConfigGroup(),
                new OTFVisConfigGroup());
			createControler(config, otfvis).run();
		} else {
			config = ConfigUtils.loadConfig(configFile, new OTFVisConfigGroup());
			Scenario scenario = ScenarioUtils.loadScenario(config);
			Controler controler = new Controler(scenario);
			
			if (otfvis) {
	            controler.addOverridingModule(new OTFVisLiveModule());
	        }
			controler.run();
		}
    }
	
	/**
	 * Function createControler
	 * -------------------------
	 * This function creates a controler based on the Configuration 
	 * specified by the {}config.xml file. 
	 * 
	 * @param config
	 * @param otfvis
	 * @return the controler
	 */
	public static Controler createControler(Config config, boolean otfvis)
    {
        TaxiConfigGroup taxiCfg = TaxiConfigGroup.get(config);
        config.addConfigConsistencyChecker(new VrpQSimConfigConsistencyChecker());
        config.checkConsistency();

        Scenario scenario = ScenarioUtils.loadScenario(config);
        TaxiData taxiData = new TaxiData();
        /* new VehicleReader(scenario.getNetwork(), taxiData).parse(taxiCfg.getTaxisFile()); */
        
        /* added the lines below to avoid a crash. Federico*/
        URL taxiCfgURL = null;
        
		try {
			System.out.println(taxiCfg.getTaxisFile());
			String FileString = new String("file://");
			taxiCfgURL = new URL(FileString.concat(taxiCfg.getTaxisFile()));
		} catch (MalformedURLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        
        new VehicleReader(scenario.getNetwork(), taxiData).parse(taxiCfgURL);
        return createControler(scenario, taxiData, otfvis, taxiCfg);
    }
	
	/**
	 * Function createControler
	 * -------------------------
	 * This function takes in more data than the other one and actually creates the
	 * controler based on the scenario, the vehicle data, and the config_group
	 * @param scenario
	 * @param taxiData
	 * @param otfvis
	 * @param taxiCfg
	 * @return the controler
	 */
	public static Controler createControler(Scenario scenario, TaxiData taxiData, boolean otfvis, TaxiConfigGroup taxiCfg)
    {
        Controler controler = new Controler(scenario);
        controler.addOverridingModule(new TaxiModule(taxiData));
        double expAveragingAlpha = 0.05;//from the AV flow paper 
        controler.addOverridingModule(
                VrpTravelTimeModules.createTravelTimeEstimatorModule( expAveragingAlpha ));
        controler.addOverridingModule(new DynQSimModule<>(AMoDQSimProvider.class));

        if (otfvis) {
            controler.addOverridingModule(new OTFVisLiveModule());
        }

        return controler;
    }
	
	/**
	 * Function main
	 * --------------
	 * MAIN METHOD: runs simulation. 
	 * @param args
	 */
	public static void main(String[] args)
    {
		boolean otfvis = false;
		boolean useAMoD = true;
		//String configFile = "./src/main/resources/configs/01c_config.xml";
		//String configFile = "./src/main/resources/amod/amod_config_short.xml";
		//String configFile = "./src/main/resources/configs/amod500_config.xml";
		//String configFile = "./src/main/resources/configs/01x01r_config.xml";
		//String configFile = "./src/main/resources/car_only/car_config.xml";
        //String configFile = "./src/main/resources/amod/amod_config.xml";
        //String configFile = "./src/main/resources/amod/amod_config_small.xml";
        //String configFile = "./src/main/resources/mielec_2014_02/config.xml";
        //String configFile = "./src/main/resources/one_taxi/one_taxi_config.xml";
		//String configFile = "./src/main/resources/configs/01x05r_config.xml";
		//String configFile = "./src/main/resources/configs/19x12r_config.xml";
		//String configFile = "./src/main/resources/configs/19x05r_config.xml";
		//String configFile = "./src/main/resources/configs/26x01r_config.xml";
		//String configFile = "./src/main/resources/configs/19x01r_config.xml";
		//String configFile = "./src/main/resources/car_only/12_car_config.xml";
		//String configFile = "./src/main/resources/seattle/seattle_config.xml";
		//String configFile = "/home/frossi2/SVN/code/AMoD_congestion/AMoD_taxi/src/main/resources/configs/01x05r_config_FOSM_24h.xml";
		//String configFile = "/home/frossi2/SVN/code/AMoD_congestion/AMoD_taxi/src/main/resources/configs/01x05r_config_FOSM_24h_25cap.xml";
		//String configFile = "/home/frossi2/SVN/code/AMoD_congestion/AMoD_taxi/src/main/resources/configs/01x05r_config_FOSM_24h_30cap.xml";
		//String configFile = "/home/frossi2/SVN/code/AMoD_congestion/AMoD_taxi/src/main/resources/configs/01x05r_config_FOSM_24h_nopax.xml";
		String configFile = "/home/frossi2/SVN/code/AMoD_congestion/AMoD_taxi/src/main/resources/configs/01x05r_config_FOSM.xml";
		//String configFile = "/home/frossi2/SVN/code/AMoD_congestion/AMoD_taxi/src/main/resources/configs/01x05r_config_Leg.xml";
        RunAMoDExample.run(configFile, otfvis, useAMoD);
    }
}
