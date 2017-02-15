import matlabcontrol.MatlabConnectionException;
import matlabcontrol.MatlabProxy;
import matlabcontrol.MatlabProxyFactory;
import matlabcontrol.MatlabProxyFactoryOptions;

/**
 * Class: MatlabConnector
 * 
 * This class contains the functionality that connects java and Matlab
 * so that the cplex optimizer can run. 
 * 
 * @author yhindy
 *
 */
public class MatlabConnector {
	
	/** Instance variables */
	private static MatlabConnector con = new MatlabConnector();
	private MatlabProxyFactoryOptions options = new MatlabProxyFactoryOptions.Builder()
			.setUsePreviouslyControlledSession(true)
			.setHidden(false)
			.setMatlabLocation(null).build();
	MatlabProxyFactory factory = new MatlabProxyFactory(options);
	
	/**
	 * constructor doesn't really do anything
	 */
	private MatlabConnector() 
	{}
	
	/**
	 * Function getProxy
	 * ------------------
	 * This function provides the proxy that can connect java to Matlab.
	 * @return the proxy that connects java to matlab. 
	 * @throws MatlabConnectionException
	 */
	public static MatlabProxy getProxy() throws MatlabConnectionException 
	{
		return con.factory.getProxy();
	}
	
}