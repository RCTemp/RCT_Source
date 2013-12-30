import java.io.IOException;
import java.net.InetAddress;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;

public class ROS2OSC extends AbstractNodeMain {

	public Publisher<std_msgs.String> publisher;
	final public static String nodename = "ros2osc" ;

	private String ocs_ip ;
	private String ocs_ad ;

	public ROS2OSC(String ocs_ip, String ocs_ad){
		super() ;
		this.ocs_ip = ocs_ip ;
		this.ocs_ad = ocs_ad ;
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(ROS2OSC.nodename);
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		this.publisher = connectedNode.newPublisher(ROS2OSC.nodename + "/test_response",
				std_msgs.String._TYPE);

		Subscriber<std_msgs.String> damage = connectedNode.newSubscriber(
				ROS2OSC.nodename + "/test_receive", std_msgs.String._TYPE);
		damage.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String emp) {
				System.out.println(emp.getData()) ;
				ROS2OSC.this.rosEcho(emp.getData()) ;
				ROS2OSC.this.oscEcho(emp.getData()) ;
			}
		});

	}

	public void rosEcho(String msg) {
		std_msgs.String str = this.publisher.newMessage();
		str.setData(msg);
		this.publisher.publish(str);
	}
	
	public void oscEcho (String msg){
		try {
		    InetAddress remoteIP  = InetAddress.getByName(this.ocs_ip);
		    int remotePort = 8000;
		    
		    OSCPortOut sender = new OSCPortOut(remoteIP, remotePort);
		    
		    String address1 = this.ocs_ad;
		    
		    Object values1[] = new Object[1];
		    values1[0] = msg;
		    
		    OSCMessage message1 = new OSCMessage(address1, values1);
		   
		    System.out.printf("Sending message1 to %s:%s at %s\n", remoteIP, remotePort, message1.getAddress());
		    sender.send(message1);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}