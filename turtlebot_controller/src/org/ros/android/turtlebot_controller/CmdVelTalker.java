package org.ros.android.turtlebot_controller;

import geometry_msgs.Vector3;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import android.view.MotionEvent;

public class CmdVelTalker extends AbstractNodeMain {

	private geometry_msgs.Twist cmd_vel;
	private Publisher<geometry_msgs.Twist> cmdVelPublisher;

	synchronized public void publish(float x, float y, float z){
		
		Vector3 vel = this.cmd_vel.getLinear() ;
		Vector3 rot = this.cmd_vel.getAngular() ;
		
		vel.setX(x) ;
		vel.setY(y) ;
		this.cmd_vel.setLinear( vel ) ;

		rot.setZ(z) ;
		this.cmd_vel.setAngular( rot ) ;
		
		this.cmdVelPublisher.publish(this.cmd_vel) ;
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("turtlebot_controller/cmd_vel_commander");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		this.cmdVelPublisher = connectedNode.newPublisher("/cmd_vel",
				geometry_msgs.Twist._TYPE);
		this.cmd_vel = connectedNode.getTopicMessageFactory().newFromType(
				geometry_msgs.Twist._TYPE);
	}
}