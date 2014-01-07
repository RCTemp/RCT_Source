package org.ros.android;

import com.google.common.base.Preconditions;

import android.app.Activity;
import android.content.ComponentName;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.DialogInterface.OnDismissListener;
import android.os.AsyncTask;
import android.os.IBinder;
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.exception.RosRuntimeException;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.net.InetAddress;
import java.net.URI;
import java.net.URISyntaxException;

public abstract class RosDialogActivity extends RosActivity {

	private SharedPreferences pref;
	private SharedPreferences.Editor editor;

	private final static String preftag = "RosDialogActivity";
	private final static String preftag_master = "ROS_MASTER_URI";
	private final static String preftag_hostname = "ROS_HOSTNAME";

	private String hostname ;
	
	protected RosDialogActivity(String notificationTicker,
			String notificationTitle) {
		super(notificationTicker, notificationTitle);
	}

	public String getHostname() {
		if (this.hostname != null) {
			return this.hostname;
		} else {
			InetAddress ros_ip;
			try {
				ros_ip = InetAddressFactory.newNonLoopback();
			} catch (RosRuntimeException e) {
				ros_ip = InetAddressFactory.newLoopback();
				e.printStackTrace();
			}
			return (this.hostname = ros_ip.getHostAddress());
		}
	}
	
	@Override
	protected void onDestroy() {
		super.onDestroy();
	}

	protected abstract void init(NodeMainExecutor nodeMainExecutor);

	@Override
	public void startMasterChooser() {

		pref = getSharedPreferences(RosDialogActivity.preftag,
				Activity.MODE_PRIVATE);

		String master_uri = pref
				.getString(RosDialogActivity.preftag_master, "");
		if (master_uri.length() == 0)
			master_uri = "http://localhost:11311";
		final MasterChooserDialog ld = new MasterChooserDialog(this,
				master_uri, getHostname());
		ld.setOnDismissListener(new OnDismissListener() {
			@Override
			public void onDismiss(DialogInterface dialog) {
				new Thread(new Runnable() {
					@Override
					public void run() {
						if (ld.isOK()) {
							String master = ld.getMasterUri();
							RosDialogActivity.this.hostname = ld.getHostname() ;
							editor = pref.edit();
							editor.putString(RosDialogActivity.preftag_master, master) ;
							editor.putString(
									RosDialogActivity.preftag_hostname,
									ld.getHostname());
							editor.commit();
							//
							if (master == null || master.length() == 0
									|| master.contains("localhost")
									|| master.contentEquals("127.0.0.1")) {
								nodeMainExecutorService.startMaster();
							} else {
								URI uri;
								try {
									uri = new URI(master);
								} catch (URISyntaxException e) {
									throw new RosRuntimeException(e);
								}
								nodeMainExecutorService.setMasterUri(uri);
							}
							new AsyncTask<Void, Void, Void>() {
								@Override
								protected Void doInBackground(Void... params) {
									RosDialogActivity.this
											.init(nodeMainExecutorService);
									return null;
								}
							}.execute();
						}
					}
				}).start();
			}
		});
		ld.show();
	}

	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		super.onActivityResult(requestCode, resultCode, data);
	}
}
