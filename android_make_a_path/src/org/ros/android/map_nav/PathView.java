/**
 * 
 */
package org.ros.android.map_nav;

import java.util.ArrayList;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.PointF;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View.MeasureSpec;
import android.widget.ImageView;

/**
 * @author y-tanaka
 *
 */
public class PathView extends ImageView {

	private Paint mPaint = new Paint(); // ペンの設定
	private int mPenColor = 0xff00ff00; // ペンの現在の選択色
	private final int THICKNESS = 10; // 太さ
	private PointF mStartPt = new PointF(); // 始点
	private PointF mEndPt = new PointF(); // 終点
	private Bitmap mBmp = null; // ViewにセットするBmp
	private Canvas mCanvas = null; // キャンバス
	
	private ArrayList<Point> path_buf; // Pointを格納するパスのバッファとなるArrayList
	
	private boolean path_q = false ; // 

	// コンストラクタ
	public PathView(Context context, AttributeSet attrs) {
		super(context, attrs);
		mPaint.setColor(mPenColor); // 色設定
		mPaint.setStrokeWidth(THICKNESS); // 太さ設定
		mPaint.setStrokeCap(Paint.Cap.ROUND); // 角を円く
		path_buf = new ArrayList<Point>(); // Pointを格納するパスのバッファとなるArrayListを初期化
	}
	
	// ViewのonMeasureをオーバーライド. Viewの位置とサイズを決める
		@Override
		protected void onMeasure(int wSpec, int hSpec) {
			super.onMeasure(wSpec, hSpec); // スーパークラスのonMeasure
			int w = MeasureSpec.getSize(wSpec); // 幅を計算
			int h = MeasureSpec.getSize(hSpec); // 高さを計算
			mBmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888); // 8bitX4の32bitでBmpをつくる
			mCanvas = new Canvas(mBmp); // Bmpをキャンバスに適用してキャンバス作成
			mCanvas.drawColor(0x0700ff00); // キャンバスをARGBで塗りつぶす. 接頭の0xは色を意味. 残り8桁はARGB
			this.setImageBitmap(mBmp); // Bmpをイメージビットマップとしてセット.
		} 
		
		// 
		public boolean switch_path_q(){
			return ( path_q = !path_q ); // path_qの切り替えスイッチ
		}

		// ViewのonTouchEventをオーバーライド.
		@Override
		public boolean onTouchEvent(MotionEvent e) {
			if ( ! path_q ) { // path_qがtrueの時のみ描画を実行
				return false ;
			} else {
				switch (e.getAction()) {
				case MotionEvent.ACTION_DOWN:
					mCanvas.drawColor(0x0700ff00, android.graphics.PorterDuff.Mode.CLEAR); // 開始毎に透明に塗り潰し
					mStartPt.set(e.getX() - 1, e.getY() - 1); // チョン押しでも描画されるように-1(drawCircleがベター)
					DrawLine(e);
					
					path_buf.clear(); // ArrayListの中身を消去
					path_buf.add( new Point((int)mStartPt.x,(int)mStartPt.y)); // path_bufにポイントを追加していく
					break;
				case MotionEvent.ACTION_HOVER_MOVE: // Hoverはカーソルが重なった状態という意味
				case MotionEvent.ACTION_MOVE:
					// 動かした時にも点を取得
					// path_buf.add(new Point((int)e.getX()-1,(int)e.getY()-1)); // 描画された点と合わせるように-1
					DrawLine(e);
					break;
				case MotionEvent.ACTION_UP:
					// mEndPt.set(e.getX(), e.getY()); // 終点を取得.
					DrawLine(e);
					break;
				}
				return true;
			}
		}
		
		// path_bufを返す
		public ArrayList<Point> getPath(){
			return path_buf ;
		}

		// 線を引く
		private void DrawLine(MotionEvent e) {
			path_buf.add( new Point((int)mStartPt.x,(int)mStartPt.y)) ;
			mEndPt.set(e.getX(), e.getY());
			mCanvas.drawLine(mStartPt.x, mStartPt.y, mEndPt.x, mEndPt.y, mPaint);
			mStartPt.set(mEndPt);
			invalidate(); // 再描画イベントを起こす
		}
}
