import java.net.URI;

import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class ROS2OSC_main {

	public static void main(String[] args) throws Exception {
		String ros_ip = null, ros_master = null, ocs_ip = null, ocs_ad = null;

		char mode = 'w';
		for (String buf : args) {
			if (buf.length() == 0) {
				System.out.println("null arg");
			} else if (buf.charAt(0) == '-' && buf.length() > 1) {
				System.out.println("option detected " + buf);
				mode = buf.charAt(1);
				if (mode == 'h') {
					System.out
							.println("[usage] command (-r ros_ip) (-m ros_master) (-o ocs_ip) (-a ocs_address)");
					System.exit(0);
				}
			} else {
				switch (mode) {
				case 'w':
					System.out.println("skip " + buf);
					break;
				case 'r':
					System.out.println("ros_ip " + buf);
					ros_ip = buf;
					break;
				case 'm':
					System.out.println("ros_master " + buf);
					ros_master = buf;
					break;
				case 'o':
					System.out.println("ocs_ip " + buf);
					ocs_ip = buf;
					break;
				case 'a':
					System.out.println("ocs_ad " + buf);
					ocs_ad = buf;
					break;
				default:
					System.out.println("unknow tag " + buf);
				}
			}
		}

		if (ros_ip == null)
			ros_ip = "157.82.4.150";
		if (ros_master == null)
			ros_master = "http://" + ros_ip + ":11311";
		if (ocs_ip == null)
			ocs_ip = ros_ip;
		if (ocs_ad == null)
			ros_ip = "/test";

		NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(
				ros_ip, new URI(ros_master));
		ROS2OSC ros2osc = new ROS2OSC(ocs_ip, ocs_ad);
		NodeMainExecutor runner = DefaultNodeMainExecutor.newDefault();
		runner.execute(ros2osc, nodeConfiguration);
	}

}
