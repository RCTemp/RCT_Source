package org.ros.android.map_nav;

import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import nav_msgs.Path; // PoseStampedの配列を生成

import javax.microedition.khronos.opengles.GL10;

import org.ros.android.view.visualization.Camera;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.layer.DefaultLayer;
import org.ros.android.view.visualization.shape.PoseShape;
import org.ros.android.view.visualization.shape.Shape;
import org.ros.namespace.GraphName; // rosjava_coreのパッケージ
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.rosjava_geometry.FrameTransformTree;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import android.content.Context;
import android.os.Handler;
import android.view.GestureDetector;
import android.view.MotionEvent;

import com.google.common.base.Preconditions;

public class MapPosePublisherLayer extends DefaultLayer {

	private static final String MAP_FRAME = "map";
	private static final String ROBOT_FRAME = "base_link";

	private final Context context;
	private Shape shape;
	private Publisher<geometry_msgs.PoseWithCovarianceStamped> initialPosePublisher;
	private Publisher<geometry_msgs.PoseStamped> androidGoalPublisher;
	private Publisher<move_base_msgs.MoveBaseActionGoal> goalPublisher;
	private Publisher<nav_msgs.Path> pathPublisher;
	private boolean visible;
	private GraphName topic;
	private GestureDetector gestureDetector;
	private Transform pose;
	private Transform fixedPose;
	private Camera camera;
	private ConnectedNode connectedNode;
	private int mode;
	private static final int POSE_MODE = 0;
	private static final int GOAL_MODE = 1;
	private static final int PATH_MODE = 2; // パス設定モード追加

	public MapPosePublisherLayer(String topic, Context context) {
		this(GraphName.of(topic), context); // 別のコンストラクタを呼ぶ
	}
	// こちらのコンストラクタで設定
	public MapPosePublisherLayer(GraphName topic, Context context) {
		this.topic = topic;
		this.context = context;
		visible = false; // 
	}

	public void setPoseMode() {
		mode = POSE_MODE;
	}

	public void setGoalMode() {
		mode = GOAL_MODE;
	}
	// PATH_MODEの設定
	public void setPathMode() {
		mode = PATH_MODE;
	}

	// OpenGL10で描画
	@Override
	public void draw(GL10 gl) {
		if (visible) {
			Preconditions.checkNotNull(pose);
			shape.draw(gl);
			// switch (mode) {
			// case PATH_MODE:
			// OpenGLでパスを描画する時はこちらを利用する.
		}
	}

	private double angle(double x1, double y1, double x2, double y2) {
		double deltaX = x1 - x2;
		double deltaY = y1 - y2;
		return Math.atan2(deltaY, deltaX);
	}
	
	// 通常はMainActivityで個別実装だが, こちらは拡張クラスなので,
	// 事前にonTouchEventを実装している. 
	// mapPosePublisherLayerがMainActivityに生成された時点で有効
	// ACTION_DOWNで有効化
	// 動かすと, ACTION_MOVEに移行
	@Override
	public boolean onTouchEvent(VisualizationView view, MotionEvent event) {
		if (visible) {
			Preconditions.checkNotNull(pose);

			Vector3 poseVector;
			Vector3 pointerVector;

			if (event.getAction() == MotionEvent.ACTION_MOVE) {
				/* switch (mode) {
				case PATH_MODE:
				// getX, getYを取得し続ける
					break;
				default:
					break;
				} */
				poseVector = pose.apply(Vector3.zero()); // (0, 0, 0)をposeにapply
				pointerVector = camera.toMetricCoordinates((int) event.getX(),
						(int) event.getY());
				// 角度計算
				double angle = angle(pointerVector.getX(),
						pointerVector.getY(), poseVector.getX(),
						poseVector.getY());
				pose = Transform.translation(poseVector).multiply(Transform.zRotation(angle));

				shape.setTransform(pose);
				return true; // 次のMotionEventに移る
			}
			if (event.getAction() == MotionEvent.ACTION_UP) {

				PoseStamped poseStamped;
				switch (mode) { // modeを切り替える
				case POSE_MODE:
					camera.setFrame(MAP_FRAME);
					poseVector = fixedPose.apply(Vector3.zero());
					pointerVector = camera.toMetricCoordinates(
							(int) event.getX(), (int) event.getY());
					double angle2 = angle(pointerVector.getX(),
							pointerVector.getY(), poseVector.getX(),
							poseVector.getY());
					fixedPose = Transform.translation(poseVector).multiply(
							Transform.zRotation(angle2));
					camera.setFrame(ROBOT_FRAME);
					poseStamped = fixedPose.toPoseStampedMessage(
							GraphName.of(ROBOT_FRAME),
							connectedNode.getCurrentTime(),
							androidGoalPublisher.newMessage());

					PoseWithCovarianceStamped initialPose = initialPosePublisher
							.newMessage();
					initialPose.getHeader().setFrameId(MAP_FRAME);
					initialPose.getPose().setPose(poseStamped.getPose());
					double[] covariance = initialPose.getPose().getCovariance();
					covariance[6 * 0 + 0] = 0.5 * 0.5;
					covariance[6 * 1 + 1] = 0.5 * 0.5;
					covariance[6 * 5 + 5] = (float) (Math.PI / 12.0 * Math.PI / 12.0);

					initialPosePublisher.publish(initialPose);
					break;
				case GOAL_MODE:
					// poseStampedの作り方
					poseStamped = pose.toPoseStampedMessage(
							GraphName.of(ROBOT_FRAME),
							connectedNode.getCurrentTime(),
							androidGoalPublisher.newMessage());
					androidGoalPublisher.publish(poseStamped);

					move_base_msgs.MoveBaseActionGoal message = goalPublisher
							.newMessage();
					message.setHeader(poseStamped.getHeader());
					message.getGoalId()
							.setStamp(connectedNode.getCurrentTime());
					message.getGoalId().setId("/move_base/move_base_client_android"
									+ connectedNode.getCurrentTime().toString());
					message.getGoal().setTargetPose(poseStamped);
					goalPublisher.publish(message);
					break;
				case PATH_MODE:
					// goalを離散化して, x, y, angleを付加して渡す
					// キャンセルされるまでループとして廻しておく…？
					// 障害物回避など, 経路計算はしない
					// eusのgo-posのような動かし方ができないか.
					// goPosPublisherを作る…？
					// 離散化して, go-posで動かす配列をつくる
					
					break;
				}
				visible = false;
				return true; // 次のMotionEventに移る
			}
		}
		gestureDetector.onTouchEvent(event); // さらにTouchEventを拾う
		return false; // 次
	}

	// 
	@Override
	public void onStart(ConnectedNode connectedNode, Handler handler,
			FrameTransformTree frameTransformTree, final Camera camera) {
		// 初期設定
		this.connectedNode = connectedNode;
		this.camera = camera;
		shape = new PoseShape(camera);
		mode = POSE_MODE;
		// ノード設定
		initialPosePublisher = connectedNode.newPublisher("/initialpose",
				"geometry_msgs/PoseWithCovarianceStamped");
		androidGoalPublisher = connectedNode.newPublisher("/android/goal",
				"geometry_msgs/PoseStamped");
		goalPublisher = connectedNode.newPublisher("/move_base/goal",
				"move_base_msgs/MoveBaseActionGoal");
		// パスをパブリッシュするノードを追加, トピック名はどうするか…
		// pathPosePublisher
		pathPublisher = connectedNode.newPublisher("/android/path", "nav_msgs/Path");
		// マルチスレッドでgestureDetectorを定義
		handler.post(new Runnable() {
			@Override
			public void run() {
				gestureDetector = new GestureDetector(context,
						new GestureDetector.SimpleOnGestureListener() {
							@Override
							public void onLongPress(MotionEvent e) { // LongPressのみに対応
								// poseの取得
								// Transformはtranslation
								// toMetricCoordinatesでx,yをタッチ座標で取得
								// zを0にして, Vector3型を得る
								// translationでさらにQuaternion.identity()を付加
								// Quaternion.identity(), (x,y,z,w) = (0,0,0,1)
								// 角度は初期化
								pose = Transform.translation(camera
										.toMetricCoordinates((int) e.getX(),
												(int) e.getY())); // zを0にして, Vector3型にする
								shape.setTransform(pose); // shapeにposeをセット
								camera.setFrame(MAP_FRAME); // tfのをmapフレームに設定
								fixedPose = Transform.translation(camera
										.toMetricCoordinates((int) e.getX(),
												(int) e.getY()));
								camera.setFrame(ROBOT_FRAME); // tfをbase_linkフレームに設定
								visible = true;
							}
							// ShortPressにどう対応するか.
							// ジェスチャはPathViewで処理して配列だけをこちらに渡す？
							// そうすると座標変換が必要になってくる…
							// 描画のパスとは別にこちらで処理？
						});
			}
		});
	}

	@Override
	public void onShutdown(VisualizationView view, Node node) {
		initialPosePublisher.shutdown();
		androidGoalPublisher.shutdown();
		goalPublisher.shutdown();
		pathPublisher.shutdown();
	}
}
