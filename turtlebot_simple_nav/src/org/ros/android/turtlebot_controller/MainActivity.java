package org.ros.android.turtlebot_controller;

import java.net.InetAddress;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.view.Display;
import android.view.WindowManager;
import android.widget.TextView;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosDialogActivity;
import org.ros.android.turtlebot_controller.R;
import org.ros.exception.RosRuntimeException;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class MainActivity extends RosDialogActivity {

	private CompressedImageView image;
	private TextView bottom_notf ;
	private CmdVelTalker talker;
	public static float width = 480, height = 640;

	public MainActivity() {
		super("turtlebot_controller", "turtlebot_controller");
	}

//	@SuppressWarnings("unchecked")
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		talker = new CmdVelTalker();

		WindowManager wm = (WindowManager) getSystemService(WINDOW_SERVICE);
		Display disp = wm.getDefaultDisplay();
		width = disp.getWidth();
		height = disp.getHeight();

		setContentView(R.layout.main);
		image = (CompressedImageView) findViewById(R.id.image);
		image.setTopicName("/camera/rgb/image_color/compressed");
		image.setMessageType(sensor_msgs.CompressedImage._TYPE);
		image.setTalker(talker) ;
		
		bottom_notf = (TextView) findViewById(R.id.bottom_notification_text) ;
	}

	@Override
	public void onResume() {
		super.onResume();
		Bitmap bmp = BitmapFactory.decodeResource(this.getResources(),
				R.drawable.robot);
		this.image.setBitmap(bmp);
	}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(
				getHostname(), getMasterUri());
		this.runOnUiThread(new Runnable() {
			public void run() {
				MainActivity.this.bottom_notf.setText(getHostname()
						+ " --[connect]--> " + getMasterUri());
			}
		});
		nodeMainExecutor.execute(image, nodeConfiguration);
		nodeMainExecutor.execute(talker, nodeConfiguration);
	}
	
}
