package com.github.omegabyte.drivesample;

/**
 * Created by lunabot on 2/2/15.
 */
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.widget.TextView;

public class PaintView extends View implements OnTouchListener{

    Paint mPaint;
    float mX;
    float mY;
    TextView mTVCoordinates;

    public PaintView(Context context,AttributeSet attributeSet){
        super(context,attributeSet);

        /** Initializing the variables */
        mPaint = new Paint();
        mX = mY = -100;
        mTVCoordinates = null;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        //setting the color of background
        //mPaint.setStyle(Paint.Style.FILL);
        //mPaint.setColor(Color.BLACK);
        //canvas.drawRect();

        // Setting the color of the circle
        mPaint.setColor(Color.GREEN);

        // Draw the circle at (x,y) with radius 15
        canvas.drawCircle(mX, mY, 15, mPaint);

        // Redraw the canvas
        invalidate();
    }

    public void setTextView(TextView tv){
        // Reference to TextView Object
        mTVCoordinates = tv;
    }

    @Override
    public boolean onTouch(View v, MotionEvent event) {
        switch(event.getAction()){
            // When user touches the screen
            case MotionEvent.ACTION_DOWN:
                // Getting X coordinate
                mX = event.getX();
                // Getting Y Coordinate
                mY = event.getY();

                // Setting the coordinates on TextView
                if(mTVCoordinates!=null){
                    mTVCoordinates.setText("X :" + mX + " , " + "Y :" + mY);
                }
        }
        return true;
    }
}
