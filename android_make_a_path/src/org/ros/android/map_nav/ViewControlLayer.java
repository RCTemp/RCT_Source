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



import java.util.concurrent.ExecutorService;
import android.annotation.SuppressLint;
import android.annotation.TargetApi;
import android.content.Context;
import android.os.Build;
import android.os.Handler;
import android.util.Log;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;
import android.view.ViewGroup;
import org.ros.android.view.RosImageView;
import org.ros.android.view.visualization.Camera;
import org.ros.android.view.visualization.RotateGestureDetector;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.layer.CameraControlLayer;
import org.ros.android.view.visualization.layer.CameraControlListener;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.node.ConnectedNode;
import org.ros.rosjava_geometry.FrameTransformTree;

/**
 * @author murase@jsk.imi.i.u-tokyo.ac.jp (Kazuto Murase)
 */
public class ViewControlLayer extends CameraControlLayer { // CameraControlLayerの拡張

    private static final String ROBOT_FRAME = "base_link";
    private final Context context;
    private final ListenerGroup<CameraControlListener> listeners;

    private GestureDetector translateGestureDetector;
    private RotateGestureDetector rotateGestureDetector;
    private ScaleGestureDetector zoomGestureDetector;

    private RosImageView<sensor_msgs.CompressedImage> cameraView;
    private VisualizationView mapView;
    private PathView pathView; // ViewControlLayerには取り込まない方がいい…？
    private ViewGroup mainLayout;
    private ViewGroup sideLayout;
    private ViewGroup mainFrameLayout;
    private ViewGroup frameLayout; // sideLayoutの代わりに使う…？
    private boolean mapViewGestureAvaiable;
    // private boolean pathViewGestureAvailable;

    // ViewModeの列挙型. CAMERA, MAPでスイッチ.
    private enum ViewMode {
	CAMERA, MAP
	    };
    private ViewMode viewMode;

    // pathViewとframeLayoutを引数にとるように改変する.
	public ViewControlLayer(Context context,
			    ExecutorService executorService,
			    RosImageView<sensor_msgs.CompressedImage> cameraView,
			    VisualizationView mapView,
			    PathView pathView,
			    ViewGroup mainLayout,
			    ViewGroup sideLayout,
			    ViewGroup mainFrameLayout,
			     ViewGroup frameLayout){
	super(context,executorService);

	this.context = context;

	listeners = new ListenerGroup<CameraControlListener>(executorService);

	this.cameraView = cameraView;
	this.mapView = mapView;
	this.mainLayout = mainLayout;
	this.sideLayout = sideLayout;
	
	// 追加
	this.pathView = pathView;
	this.mainFrameLayout = mainFrameLayout;
	this.frameLayout = frameLayout; // mapViewもpathViewも現在はここに入っている.

	viewMode = ViewMode.CAMERA;
	this.cameraView.setOnClickListener(new View.OnClickListener() {
		@Override
		    public void onClick(View v){
		    swapViews();
		}
	    });
	// 小さい方をClickableにする. mapViewがClickableで
	this.mapView.setClickable(true);
	// this.pathView.setClickable(true); // 未使用
	// this.pathView.setVisibility(View.GONE);
	this.cameraView.setClickable(false);
	mapViewGestureAvaiable = false; // デフォルト値
    }


    @Override
	public boolean onTouchEvent(VisualizationView view,MotionEvent event){

	if(event.getAction()==MotionEvent.ACTION_UP){
	    mapViewGestureAvaiable = true;
	}
	if(viewMode == ViewMode.CAMERA){
	    swapViews();
	    return true;
	}
	else {
	    if (translateGestureDetector == null || rotateGestureDetector == null
		|| zoomGestureDetector == null) {
		return false;
	    }
	    return translateGestureDetector.onTouchEvent(event)
		|| rotateGestureDetector.onTouchEvent(event) || zoomGestureDetector.onTouchEvent(event);
	}
    }



    /**
     * Swap the camera and map views.
     */
    private void swapViews() {
	// Figure out where the views were...
	ViewGroup mapViewParent; // スイッチするためのtmp変数
	ViewGroup cameraViewParent; // 同上
	// ViewGroup pathViewParent; // pathView用のレイアウト

	if (viewMode == ViewMode.CAMERA) { // メイン画面がカメラの時

	    // mapViewParent = sideLayout;
		mapViewParent = frameLayout;
	    // pathViewParent = frameLayout;
	    // cameraViewParent = mainLayout;
		cameraViewParent = mainFrameLayout;
	} else { // ViewModeがマップの時のレイアウト

	    mapViewParent = mainFrameLayout;
	    // paintViewParent = mainFrameLayout;
	    // cameraViewParent = sideLayout;
	    cameraViewParent = frameLayout;
	}
	int mapViewIndex = mapViewParent.indexOfChild(mapView); // viewの中の何番目のchildか
	int cameraViewIndex = cameraViewParent.indexOfChild(cameraView);
	// int pathViewIndex = mapViewParent.indexOfChild(pathView);
	Log.d("mapViewIndex", Integer.toString(mapViewIndex)); // 0
	Log.d("cameraViewIndex", Integer.toString(cameraViewIndex)); // 0
	// Log.d("pathViewIndex", Integer.toString(pathViewIndex)); // 1
	
	// Remove the views from their old locations...
	mapViewParent.removeView(pathView); // 上からremoveViewする.
	mapViewParent.removeView(mapView); // mapViewParentがflameLayoutになっていれば、そこからremoveされる.
	cameraViewParent.removeView(cameraView);

	// Add them to their new location...
	mapViewParent.addView(cameraView, mapViewIndex); // 0
	cameraViewParent.addView(mapView, cameraViewIndex); // 0
	cameraViewParent.addView(pathView, cameraViewIndex+1);

	// Remeber that we are in the other mode now.
	if (viewMode == ViewMode.CAMERA) {
	    viewMode = ViewMode.MAP;
	    mapViewGestureAvaiable = false; // ?
	} else {
	    viewMode = ViewMode.CAMERA;
	}
	mapView.getCamera().jumpToFrame(ROBOT_FRAME);
	mapView.setClickable(viewMode != ViewMode.MAP);
	cameraView.setClickable(viewMode != ViewMode.CAMERA);

    }

    // レイヤがナビゲーションビューに登録された時に呼ばれる.
    // 実際にモーションイベントを処理するコードを書く.
    @Override
	public void onStart(ConnectedNode connectedNode, Handler handler,
			    FrameTransformTree frameTransformTree, final Camera camera) {
	handler.post(new Runnable() {
		@Override
		    public void run() {
		    translateGestureDetector =
			new GestureDetector(context, new GestureDetector.SimpleOnGestureListener() {
				@Override
				    public boolean onScroll(MotionEvent event1, MotionEvent event2,
							    final float distanceX, final float distanceY) {
					// should be modified by y-tanaka
				    if (mapViewGestureAvaiable) {
				    	// CameraControlLayerの拡張なので, cameraが使われている.
				    	camera.translate(-distanceX, distanceY);
				    	listeners.signal(new SignalRunnable<CameraControlListener>() {
				    		@Override
						    public void run(CameraControlListener listener) {
						    listener.onTranslate(-distanceX, distanceY);
						}
					    });
				    return true;
				    } // end point

				    return false;
				}
			    });
		    rotateGestureDetector =
			new RotateGestureDetector(new RotateGestureDetector.OnRotateGestureListener() {
				@Override
				    public boolean onRotate(MotionEvent event1, MotionEvent event2,
							    final double deltaAngle) {
				    if (mapViewGestureAvaiable) {
					final double focusX = (event1.getX(0) + event1.getX(1)) / 2;
					final double focusY = (event1.getY(0) + event1.getY(1)) / 2;
					camera.rotate(focusX, focusY, deltaAngle);
					listeners.signal(new SignalRunnable<CameraControlListener>() {
						@Override
						    public void run(CameraControlListener listener) {
						    listener.onRotate(focusX, focusY, deltaAngle);
						}
					    });
				    // Don't consume this event in order to allow the zoom gesture
				    // to also be detected.
					return false;
				    }

				    return true;
				}
			    });
		    zoomGestureDetector =
			new ScaleGestureDetector(context,
						 new ScaleGestureDetector.SimpleOnScaleGestureListener() {
						     @Override
							 public boolean onScale(ScaleGestureDetector detector) {
							 if (!detector.isInProgress()) {
							     return false;
							 }
							 if (mapViewGestureAvaiable) {
							     final float focusX = detector.getFocusX();
							     final float focusY = detector.getFocusY();
							     final float factor = detector.getScaleFactor();
							     camera.zoom(focusX, focusY, factor);
							     listeners.signal(new SignalRunnable<CameraControlListener>() {
								     @Override
									 public void run(CameraControlListener listener) {
									 listener.onZoom(focusX, focusY, factor);
								     }
								 });
							     return true;
							 }

							 return false;
						     }
						 });
		}
	    });
    }
}