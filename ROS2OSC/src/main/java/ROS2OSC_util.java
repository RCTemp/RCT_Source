//import geometry_msgs.Point;
//import geometry_msgs.PoseWithCovariance;

import java.io.IOException;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.List;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import ROS2OSC.osc;

import com.illposed.osc.OSCMessage;
import com.illposed.osc.OSCPortOut;

public class ROS2OSC_util extends AbstractNodeMain {

	public Publisher<std_msgs.String> publisher;
	final public static String nodename = "ros2osc"  ;

	private String ocs_ip ;
	private String ocs_ad ;
	private String topic ;

	private HashMap<String,OSCPortOut> sender ;
	
	public ROS2OSC_util(String ocs_ip, String ocs_ad, String topic){
		super() ;
		this.ocs_ip = ocs_ip ;
		this.ocs_ad = ocs_ad ;
		this.topic = topic ;
		
		this.sender = new HashMap<String,OSCPortOut>() ;
	}

	public OSCPortOut connect(String osc_ip, int osc_port){
		InetAddress remoteIP;
		if ( this.sender.containsKey(osc_ip+":"+osc_port) ){
			return this.sender.get(osc_ip+":"+osc_port) ;
		} else {
		    log(String.format("new connection to %s:%s\n", osc_ip, osc_port));
			try {
				remoteIP = InetAddress.getByName(osc_ip);
				int remotePort = osc_port;
				OSCPortOut com = new OSCPortOut(remoteIP, remotePort);
				this.sender.put(osc_ip+":"+osc_port, com) ;
				return com ;
			} catch (UnknownHostException e) {
				e.printStackTrace();
				return null ;
			} catch (SocketException e) {
				e.printStackTrace();
				return null ;
			}
		}
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(ROS2OSC_util.nodename + System.currentTimeMillis());
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		this.publisher = connectedNode.newPublisher(ROS2OSC_util.nodename + "/" + "feedback",
				std_msgs.String._TYPE);

//		Subscriber<std_msgs.String> sr = connectedNode.newSubscriber(
//				ROS2OSC.nodename + "/string/" + this.topic , std_msgs.String._TYPE);
//		sr.addMessageListener(new MessageListener<std_msgs.String>() {
//			@Override
//			public void onNewMessage(std_msgs.String emp) {
//				System.out.println(emp.getData()) ;
//				ROS2OSC.this.rosEcho(emp.getData()) ;
//				ROS2OSC.this.oscEcho(emp.getData()) ;
//			}
//		});
//
//		Subscriber<std_msgs.Float32> fl = connectedNode.newSubscriber(
//				ROS2OSC.nodename + "/float/" + this.topic , std_msgs.Float32._TYPE);
//		fl.addMessageListener(new MessageListener<std_msgs.Float32>() {
//			@Override
//			public void onNewMessage(std_msgs.Float32 emp) {
//				System.out.println(emp.getData()) ;
//				ROS2OSC.this.rosEcho(emp.getData()+"") ;
//				ROS2OSC.this.oscEcho(emp.getData()) ;
//			}
//		});

		Subscriber<std_msgs.String> util = connectedNode.newSubscriber(
				ROS2OSC_util.nodename + "/string/" + this.topic , std_msgs.String._TYPE);
		util.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String emp) {
				ROS2OSC_util.this.oscEcho(emp.getData()) ;
			}
		});

		Subscriber<ROS2OSC.ros2osc> util2 = connectedNode.newSubscriber(
				ROS2OSC_util.nodename + "/" + this.topic , ROS2OSC.ros2osc._TYPE);
		util2.addMessageListener(new MessageListener<ROS2OSC.ros2osc>() {
			@Override
			public void onNewMessage(ROS2OSC.ros2osc msg) {
				ROS2OSC_util.this.oscEcho(msg) ;
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
	
	public void oscEcho( ROS2OSC.ros2osc msg ){
		String ip = strErr(msg.getOscIp(), "127.0.0.1") ;
		String port = strErr(msg.getOscPort(), "8000") ;
		String add = strErr(msg.getOscAddress(), "/test") ;
		List<ROS2OSC.osc> msgs = msg.getOscMessages() ;
		//
		OSCPortOut com = connect(ip, Integer.parseInt(port)) ;
		Object[] msgs2 = new Object[msgs.size()] ;
		for ( int i=0 ; i<msgs2.length ; i++ ){
			ROS2OSC.osc osc = msgs.get(i) ;
			String type = osc.getType().toLowerCase() ;
			if ( type.contains("int") ){
				msgs2[i] = osc.getInt() ;
			} else if ( type.contains("float") ){
				msgs2[i] = osc.getFloat() ;
			} else if ( type.contains("str") ){
				msgs2[i] = osc.getStr() ;
			} 
		}
		//
		try {
		    OSCMessage message = new OSCMessage(add, msgs2);
		    log(String.format("Sending msg to %s:%s %s at %s\n", ip, port, objString(msgs2), add));
		    com.send(message);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public void oscEcho(Object msg){
		oscEcho(this.ocs_ad, new Object[]{msg}) ;
	}
	
	public void oscEcho(String s){
		String[] buf = s.split(" ") ;
		if ( buf.length < 1 ){
			log(String.format("invalid string %s\n", s));
			return ;
		}
		String adr = buf[0] ;
		Object[] msg = new Object[buf.length-1] ;
		Object[] tmp = new Object[1] ;
		for ( int i=1 ; i<buf.length ; i++ ){
			if ( ROS2OSC_util.parseFloat(buf[i], tmp) ){
				msg[i-1] = tmp[0] ;
			} else if ( ROS2OSC_util.parseBoolean(buf[i], tmp) ){
				msg[i-1] = tmp[0] ;
			} else {
				msg[i-1] = buf[i] ;
			}
		}
		oscEcho(adr, msg) ;
	}
	
	public void oscEcho (String ocs_ad, Object[] msg){
		OSCPortOut com = connect(this.ocs_ip, 8000) ;
		try {
			String address1 = ocs_ad;
		    OSCMessage message1 = new OSCMessage(address1, msg);
		   
		    log(String.format("Sending msg to %s:%s %s at %s\n", this.ocs_ip, 8000, objString(msg), ocs_ad));
		    com.send(message1);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public String strErr( String org, String defo ){
		if ( org == null || org.length() == 0 ){
			return defo ;
		} else {
			return org ;
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
	
	public void log(String log){
		System.out.println( "[ROS2OSC] " + log ) ;
		rosEcho("[ROS2OSC] " + log ) ;
	}
	
	public static String objString(Object[] obj){
		if ( obj == null || obj.length == 0 ) return "" ;
		StringBuilder ret = new StringBuilder() ;
		for ( Object o : obj ){
			if ( o.getClass() == String.class ){
				ret.append ( "\"" + o + "\"" ) ;
			} else if ( o.getClass() == Float.class ){
				ret.append("(float)" + o) ;
			} else if ( o.getClass() == Integer.class ){
				ret.append("(int)" + o) ;
			} else if ( o.getClass() == Boolean.class ){
				ret.append("(boolean)" + o) ;
			}
			ret.append("/") ;
		}
		return ret.substring(0, ret.length()-1) ;
	}
}