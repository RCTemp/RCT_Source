import java.io.IOException;
import java.net.InetAddress;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;

public class ROS2OSC extends AbstractNodeMain {

	public Publisher<std_msgs.String> publisher;
	final public static String nodename = "ros2osc" ;

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
		    InetAddress remoteIP  = InetAddress.getByName("157.82.6.123");
			//InetAddress remoteIP = InetAddress.getLocalHost();
		    int remotePort = 8000;
		    
		    OSCPortOut sender = new OSCPortOut(remoteIP, remotePort);
		    
		    // The address to send our message to
		    String address1 = "/test";
		    
		    // An array of objects that are our values we would like to send
		    Object values1[] = new Object[1];
		    //values1[0] = new Integer(3);
		    values1[0] = msg;
		    
		    // Bring the address and values together to form an OSCMessage
		    OSCMessage message1 = new OSCMessage(address1, values1);
		   
		    // Send each message
		    System.out.printf("Sending message1 to %s:%s at %s\n", remoteIP, remotePort, message1.getAddress());
		    sender.send(message1);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}