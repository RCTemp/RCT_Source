//import geometry_msgs.Point;
//import geometry_msgs.PoseWithCovariance;

import java.io.IOException;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

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
	final public static String nodename = "ros2osc"  ;

	private String ocs_ip ;
	private String ocs_ad ;
	private String topic ;

	private OSCPortOut sender ;
	
	public ROS2OSC(String ocs_ip, String ocs_ad, String topic){
		super() ;
		this.ocs_ip = ocs_ip ;
		this.ocs_ad = ocs_ad ;
		this.topic = topic ;
		
		if ( this.sender == null ){
			InetAddress remoteIP;
			try {
				remoteIP = InetAddress.getByName(this.ocs_ip);
				int remotePort = 8000;
				this.sender = new OSCPortOut(remoteIP, remotePort);
			} catch (UnknownHostException e) {
				this.sender = null ;
				e.printStackTrace();
			} catch (SocketException e) {
				this.sender = null ;
				e.printStackTrace();
			}
		}
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(ROS2OSC.nodename + System.currentTimeMillis());
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		this.publisher = connectedNode.newPublisher(ROS2OSC.nodename + "/" + "test_feedback",
				std_msgs.String._TYPE);

		Subscriber<std_msgs.String> sr = connectedNode.newSubscriber(
				ROS2OSC.nodename + "/string/" + this.topic , std_msgs.String._TYPE);
		sr.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String emp) {
				System.out.println(emp.getData()) ;
				ROS2OSC.this.rosEcho(emp.getData()) ;
				ROS2OSC.this.oscEcho(emp.getData()) ;
			}
		});

		Subscriber<std_msgs.Float32> fl = connectedNode.newSubscriber(
				ROS2OSC.nodename + "/float/" + this.topic , std_msgs.Float32._TYPE);
		fl.addMessageListener(new MessageListener<std_msgs.Float32>() {
			@Override
			public void onNewMessage(std_msgs.Float32 emp) {
				System.out.println(emp.getData()) ;
				ROS2OSC.this.rosEcho(emp.getData()+"") ;
				ROS2OSC.this.oscEcho(emp.getData()) ;
			}
		});

		Subscriber<std_msgs.String> util = connectedNode.newSubscriber(
				ROS2OSC.nodename + "/" + this.topic , std_msgs.String._TYPE);
		util.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String emp) {
				System.out.println(emp.getData()) ;
				ROS2OSC.this.rosEcho(emp.getData()) ;
				ROS2OSC.this.oscEcho(emp.getData()) ;
			}
		});
		
//		Subscriber<nav_msgs.Odometry> odom = connectedNode.newSubscriber(
//				"/odom" , nav_msgs.Odometry._TYPE);
//		odom.addMessageListener(new MessageListener<nav_msgs.Odometry>() {
//			@Override
//			public void onNewMessage(nav_msgs.Odometry emp) {
//				Point pos = emp.getPose().getPose().getPosition();
//				System.out.println(pos) ;
//				ROS2OSC.this.rosEcho(pos + "") ;
//				ROS2OSC.this.oscEcho("/ballx",pos.getX()) ;
//				ROS2OSC.this.oscEcho("/bally",pos.getY()) ;
//			}
//		});
	}

	public void rosEcho(String msg) {
		std_msgs.String str = this.publisher.newMessage();
		str.setData(msg);
		this.publisher.publish(str);
	}
	
	public void oscEcho(Object msg){
		oscEcho(this.ocs_ad, new Object[]{msg}) ;
	}
	
	public void oscEcho(String s){
		String[] buf = s.split(" ") ;
		if ( buf.length < 1 ){
			System.out.printf("[ROS2OSC] invalid string %s\n", s);
			return ;
		}
		String adr = buf[0] ;
		Object[] msg = new Object[buf.length-1] ;
		Object[] tmp = new Object[1] ;
		for ( int i=1 ; i<buf.length ; i++ ){
			if ( ROS2OSC.parseFloat(buf[i], tmp) ){
				msg[i-1] = tmp[0] ;
			} else if ( ROS2OSC.parseBoolean(buf[i], tmp) ){
				msg[i-1] = tmp[0] ;
			} else {
				msg[i-1] = buf[i] ;
			}
		}
		oscEcho(adr, msg) ;
	}
	
	public void oscEcho (String ocs_ad, Object[] msg){
		if ( this.sender == null ){
			InetAddress remoteIP;
			try {
				remoteIP = InetAddress.getByName(this.ocs_ip);
				int remotePort = 8000;
				this.sender = new OSCPortOut(remoteIP, remotePort);
			} catch (UnknownHostException e) {
				this.sender = null ;
				e.printStackTrace();
			} catch (SocketException e) {
				this.sender = null ;
				e.printStackTrace();
			}
		}
		try {
			String address1 = ocs_ad;
		    OSCMessage message1 = new OSCMessage(address1, msg);
		   
		    System.out.printf("[ROS2OSC] Sending message1 to %s:%s %s at %s\n", this.ocs_ip, 8000, objString(msg), ocs_ad);
		    this.sender.send(message1);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public static boolean parseFloat(String target, Object[] ret){
		try {
			Float f = Float.parseFloat(target) ;
			ret[0] = f ;
			return true ;
		} catch ( NumberFormatException e ){
			//e.printStackTrace() ;
			return false ;
		}
	}
	
	public static boolean parseBoolean(String target, Object[] ret){
		target = target.toLowerCase().trim() ;
		if ( target.contains("true") ){
			ret[0] = (Boolean)true ;
			return true ;
		} else if ( target.contains("false") ){
			ret[0] = (Boolean)false ;
			return true ;
		} else {
			return false ;
		}
	}
	
	public static String objString(Object[] obj){
		if ( obj == null || obj.length == 0 ) return "" ;
		StringBuilder ret = new StringBuilder() ;
		for ( Object o : obj ){
			if ( o.getClass() == String.class ){
				ret.append ( "\"" + o + "\"" ) ;
			} else if ( o.getClass() == Float.class ){
				ret.append(o) ;
			} else if ( o.getClass() == Boolean.class ){
				ret.append("?" + o) ;
			}
			ret.append("/") ;
		}
		return ret.substring(0, ret.length()-1) ;
	}
}