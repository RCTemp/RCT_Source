import java.net.URI;

import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class ROS2OSC_main {

	public static void main(String[] args) throws Exception {
		String client = "157.82.4.150" ; //"133.11.216.39";  
		if (args.length > 0) {
			client = args[0];
		}
		String master =  "http://" + client + ":11311" ; //"http://10.0.0.51:11311"; 
		if (args.length > 1) {
			master = args[1];
		}
		NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(
				client, new URI(master));
		ROS2OSC ros2osc = new ROS2OSC() ;
		NodeMainExecutor runner = DefaultNodeMainExecutor.newDefault();
		runner.execute(ros2osc, nodeConfiguration);
	}
}
