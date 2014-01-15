/*
 * Copyright (C) 2013 OSRF.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.android.map_nav;

import java.sql.Date;
import java.text.DateFormat;
import java.util.List;
import java.util.concurrent.TimeUnit;

import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.DialogInterface;
import android.graphics.Point;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
// import android.widget.RadioButton; // 不要
import map_store.ListMapsResponse;
import map_store.MapListEntry;
import map_store.PublishMapResponse;

// import org.ros.android.map_manager.PaintView;
// import org.ros.android.map_manager.R;
import org.ros.android.robotapp.RosAppActivity;
import org.ros.android.view.RosImageView;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceResponseListener;
import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.view.VirtualJoystickView;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.layer.CameraControlListener;
import org.ros.android.view.visualization.layer.OccupancyGridLayer;
import org.ros.android.view.visualization.layer.PathLayer;
import org.ros.android.view.visualization.layer.PoseSubscriberLayer;
import org.ros.exception.RemoteException;
import org.ros.time.NtpTimeProvider;
import org.ros.android.map_nav.StringPublisher; // 追加

/**
 * @author murase@jsk.imi.i.u-tokyo.ac.jp (Kazuto Murase)
 */
public class MainActivity extends RosAppActivity {

	private static final String MAP_FRAME = "map";
	private static final String ROBOT_FRAME = "base_link";
	private static final String cameraTopic = "camera/rgb/image_color/compressed_throttle";

	private RosImageView<sensor_msgs.CompressedImage> cameraView;
	private VirtualJoystickView virtualJoystickView;
	private VisualizationView mapView;
	private ViewGroup mainLayout;
	private ViewGroup sideLayout;
	private Button backButton;
	private Button chooseMapButton;
	// RadioGroupとRadioButtonはコードで実装されていない. setPoseClickedメソッド等との紐付けのみ.
	private MapPosePublisherLayer mapPosePublisherLayer;
	private ProgressDialog waitingDialog;
	private AlertDialog chooseMapDialog;
	private NodeMainExecutor nodeMainExecutor;
	private NodeConfiguration nodeConfiguration;
	
	// 新たに定義されたフィールド
	// private RadioButton setPathButton; // onClickでメソッドを実装しているため, 不要.
	private Button sendButton;
	private PathView pathView;
	private ViewGroup mainFrameLayout;
	private ViewGroup frameLayout;
	private StringPublisher stringPub; // pathをstring型でpublish
	// private PathPublisher pathPublisher;　
	
	// 1. MainActivity
	public MainActivity() {
		// The RosActivity constructor configures the notification title and
		// ticker
		// messages.
		super("Map nav", "Map nav");
	}

	// 2. onCreate - アプリ起動時に呼ばれる. さらに先にコンストラクタが呼ばれる.
	@SuppressWarnings("unchecked")
	@Override
	public void onCreate(Bundle savedInstanceState) {
		// LogCatのテスト
		Log.d("タグ", ",メッセージ");

		String defaultRobotName = getString(R.string.default_robot);
		String defaultAppName = getString(R.string.default_app);
		setDefaultRobotName(defaultRobotName);
		setDefaultAppName(defaultAppName);
		setDashboardResource(R.id.top_bar);
		setMainWindowResource(R.layout.main); // レイアウトの適用
		super.onCreate(savedInstanceState); // 最初に呼ばれる

		cameraView = (RosImageView<sensor_msgs.CompressedImage>) findViewById(R.id.image);
		cameraView.setMessageType(sensor_msgs.CompressedImage._TYPE);
		cameraView.setMessageToBitmapCallable(new BitmapFromCompressedImage());
		mapView = (VisualizationView) findViewById(R.id.map_view);
		virtualJoystickView = (VirtualJoystickView) findViewById(R.id.virtual_joystick);
		virtualJoystickView.setTopicName("/cmd_vel");
		backButton = (Button) findViewById(R.id.back_button);
		chooseMapButton = (Button) findViewById(R.id.choose_map_button);
		// 新たに追加されたオブジェクト
		// setPathButton = (RadioButton) findViewById(R.id.set_path_button); // onClickでのみ利用.
		sendButton = (Button) findViewById(R.id.send_button); // sendButtonのレイアウト適用
		pathView = (PathView) findViewById(R.id.path_view); // paintViewのレイアウト適用
		// stringPub = new StringPublisher() ; // stringPublisherオブジェクト生成

		backButton.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				onBackPressed();
			}
		});

		chooseMapButton.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				onChooseMapButtonPressed();
			}
		});
		
		// sentButtonのonClickListener
		// MapPosePublisherLayerに渡して処理するべき?
		// PathViewで取得した座標をMapPosePublisherLayerに渡す.
		sendButton.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {
				// mapPosePublisherLayerのメソッドを動作させるための条件を整える.
				// mapPosePublisherLayer.setVisible(true);
				// mapPosePublisherLayer.setPathMode();
				// visible == trueとしている間にpathを送る.
				mapPosePublisherLayer.echoPath(pathView.getPath());
				// path送信後, visible == falseとする.
				// mapPosePublisherLayer.setVisible(false);
				/* StringBuilder path = new StringBuilder();
				for ( Point p : pathView.getPath() ){
					path.append( "(" + p.x + "," + p.y + ") " ) ;
				}
				String ec = path.substring(0,path.length()-1) ;
				System.out.println(ec);
				stringPub.echo(ec); */
			}
		});

		mapView.getCamera().jumpToFrame(ROBOT_FRAME);
		mainLayout = (ViewGroup) findViewById(R.id.main_layout);
		sideLayout = (ViewGroup) findViewById(R.id.side_layout);
		mainFrameLayout = (ViewGroup) findViewById(R.id.main_frame_layout);
		frameLayout = (ViewGroup) findViewById(R.id.frame_layout);

	}

	// 3. rosjavaのノードの初期化, ViewControlレイヤの設定など.
	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {

		super.init(nodeMainExecutor);
		
		this.nodeMainExecutor = nodeMainExecutor;
		nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory
				.newNonLoopback().getHostAddress(), getMasterUri());

		NameResolver appNameSpace = getAppNameSpace();
			cameraView.setTopicName(appNameSpace.resolve(cameraTopic)
					.toString());
			
		nodeMainExecutor.execute(cameraView,
				nodeConfiguration.setNodeName("android/camera_view"));
		nodeMainExecutor.execute(virtualJoystickView,
				nodeConfiguration.setNodeName("android/virtual_joystick"));

		// paintViewとframeLayoutを追加でわたす. Context:共有される領域.
		ViewControlLayer viewControlLayer = new ViewControlLayer(this, // Context(アプリ情報)をわたす.
				nodeMainExecutor.getScheduledExecutorService(), cameraView,
				mapView, pathView, mainLayout, sideLayout, mainFrameLayout, frameLayout);

		// モーションのイベントリスナー追加
		viewControlLayer.addListener(new CameraControlListener() {
			@Override
			public void onZoom(double focusX, double focusY, double factor) {

			}

			@Override
			public void onTranslate(float distanceX, float distanceY) {

			}

			@Override
			public void onRotate(double focusX, double focusY, double deltaAngle) {

			}
		});

		// mapViewに必要なレイヤを追加. map上に上手くパスを送るためには, ここに追加する必要がある…？
		mapView.addLayer(viewControlLayer);
		mapView.addLayer(new OccupancyGridLayer("map"));
		mapView.addLayer(new PathLayer("/move_base/NavfnROS/plan"));
		mapPosePublisherLayer = new MapPosePublisherLayer("pose", this);
		mapView.addLayer(mapPosePublisherLayer);
		mapView.addLayer(new InitialPoseSubscriberLayer("/android/initialpose"));
		mapView.addLayer(new PoseSubscriberLayer("/android/goal"));
		mapView.addLayer(new PoseSubscriberLayer("/android/path_pose"));
		NtpTimeProvider ntpTimeProvider = new NtpTimeProvider(
				InetAddressFactory.newFromHostString("192.168.0.1"),
				nodeMainExecutor.getScheduledExecutorService());
		ntpTimeProvider.startPeriodicUpdates(1, TimeUnit.MINUTES);
		nodeConfiguration.setTimeProvider(ntpTimeProvider);
		nodeMainExecutor.execute(mapView, nodeConfiguration.setNodeName("android/map_view"));

		readAvailableMapList(); // 起動時にマップを表示
	}

	// chooseMapButton.setOnClickListenerから呼ばれる
	private void onChooseMapButtonPressed() {
		readAvailableMapList();
	}
	
	// PoseとGoalの設定
	public void setPoseClicked(View view) {
		setPose();
	}
	private void setPose() {
		mapPosePublisherLayer.setPoseMode();
	}
	public void setGoalClicked(View view) {
		setGoal();
	}
	private void setGoal() {
		mapPosePublisherLayer.setGoalMode();
	}
	
	// Pathの設定. PoseとGoalと同じように実装. PaintViewへの操作委譲など.
	public void setPathClicked(View view) {
		setPath();
	}
	// PathViewの呼び出し
	private void setPath() {
		// setPoseModeの拡張？
		// Pathモードへの切り替え. VISIBLEとGONEの切り替え.
		// ここではPathViewのみ有効化
		Log.d("setPath()", "パスモードへ切替");
		if (pathView.getVisibility() == View.VISIBLE) {
			pathView.setVisibility(View.GONE);
		} else {
			Log.d("pathView", "Visible with light green color.");
			pathView.switch_path_q();
			pathView.setVisibility(View.VISIBLE);
			// mapView.setVisibility(View.GONE);
		}
	}

	// MapManagerの機能を利用して地図を読み込むメソッド
	private void readAvailableMapList() {
		safeShowWaitingDialog("Waiting...", "Waiting for map list");

		MapManager mapManager = new MapManager(); // MapManagerオブジェクト生成
		mapManager.setFunction("list");
		safeShowWaitingDialog("Waiting...", "Waiting for map list");
		mapManager.setListService(new ServiceResponseListener<ListMapsResponse>() {
					@Override
					public void onSuccess(ListMapsResponse message) {
						Log.i("MapNav", "readAvailableMapList() Success");
						safeDismissWaitingDialog();
						showMapListDialog(message.getMapList());
					}

					@Override
					public void onFailure(RemoteException e) {
						Log.i("MapNav", "readAvailableMapList() Failure");
						safeDismissWaitingDialog();
					}
				});

		nodeMainExecutor.execute(mapManager,
				nodeConfiguration.setNodeName("android/list_maps"));
	}

	/**
	 * Show a dialog with a list of maps. Safe to call from any thread.
	 */
	// readAvailabelMapList()内で利用
	private void showMapListDialog(final List<MapListEntry> list) {
		// Make an array of map name/date strings.
		final CharSequence[] availableMapNames = new CharSequence[list.size()];
		for (int i = 0; i < list.size(); i++) {
			String displayString;
			String name = list.get(i).getName();
			Date creationDate = new Date(list.get(i).getDate() * 1000);
			String dateTime = DateFormat.getDateTimeInstance(DateFormat.MEDIUM,
					DateFormat.SHORT).format(creationDate);
			if (name != null && !name.equals("")) {
				displayString = name + " " + dateTime;
			} else {
				displayString = dateTime;
			}
			availableMapNames[i] = displayString;
		}

		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				AlertDialog.Builder builder = new AlertDialog.Builder(
						MainActivity.this);
				builder.setTitle("Choose a map");
				builder.setItems(availableMapNames,
						new DialogInterface.OnClickListener() {
							@Override
							public void onClick(DialogInterface dialog,
									int itemIndex) {
								loadMap(list.get(itemIndex));
							}
						});
				chooseMapDialog = builder.create();
				chooseMapDialog.show();
			}
		});
	}

	// showMapListDialog内で利用
	private void loadMap(MapListEntry mapListEntry) {

		MapManager mapManager = new MapManager();
		mapManager.setFunction("publish");
		mapManager.setMapId(mapListEntry.getMapId());

		safeShowWaitingDialog("Waiting...", "Loading map");
		try {
			mapManager
					.setPublishService(new ServiceResponseListener<PublishMapResponse>() {
						@Override
						public void onSuccess(PublishMapResponse message) {
							Log.i("MapNav", "loadMap() Success");
							safeDismissWaitingDialog();
							// poseSetter.enable();
						}

						@Override
						public void onFailure(RemoteException e) {
							Log.i("MapNav", "loadMap() Failure");
							safeDismissWaitingDialog();
						}
					});
		} catch (Throwable ex) {
			Log.e("MapNav", "loadMap() caught exception.", ex);
			safeDismissWaitingDialog();
		}
		nodeMainExecutor.execute(mapManager,
				nodeConfiguration.setNodeName("android/publish_map"));
	}

	// 使用されていない?
	private void safeDismissChooseMapDialog() {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				if (chooseMapDialog != null) {
					chooseMapDialog.dismiss();
					chooseMapDialog = null;
				}
			}
		});
	}

	// safeShowWaitingDialog内で使用
	private void showWaitingDialog(final CharSequence title,
			final CharSequence message) {
		dismissWaitingDialog();
		waitingDialog = ProgressDialog.show(MainActivity.this, title, message,
				true);
		waitingDialog.setProgressStyle(ProgressDialog.STYLE_SPINNER);
	} // ダイアログの表示
	private void safeShowWaitingDialog(final CharSequence title,
			final CharSequence message) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				showWaitingDialog(title, message);
			}
		});
	}

	// safeDismissWatingDialog内で使用
	private void dismissWaitingDialog() {
		if (waitingDialog != null) {
			waitingDialog.dismiss();
			waitingDialog = null;
		}
	} // ダイアログの表示
	private void safeDismissWaitingDialog() {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				dismissWaitingDialog();
			}
		});
	}

	// メニュー関連
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		menu.add(0, 0, 0, R.string.stop_app);
		return super.onCreateOptionsMenu(menu);
	}
	// オプション関連
	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		super.onOptionsItemSelected(item);
		switch (item.getItemId()) {
		case 0:
			onDestroy();
			break;
		}
		return true;
	}
}