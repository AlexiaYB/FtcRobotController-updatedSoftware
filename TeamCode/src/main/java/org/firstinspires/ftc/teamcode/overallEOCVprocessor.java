package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class overallEOCVprocessor implements VisionProcessor{
    public Rect rectLeft;
    public  Rect rectRight;
    public boolean red;

    public double distanceRectLeft;
    public double distanceRectRight;

    public double hueLeft;
    public double hueRight;

    Selected selection = Selected.NONE;
    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    public overallEOCVprocessor(Rect rectLeftInput, Rect rectRightInput, boolean redTrue){
        rectLeft = rectLeftInput;
        rectRight = rectRightInput;
        red = redTrue;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {;
    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        distanceRectLeft = getColourDistance(hsvMat, rectLeft, true);
        distanceRectRight = getColourDistance(hsvMat, rectRight, false);
        if (Math.abs(distanceRectLeft - distanceRectRight ) < 3){
            return Selected.NONE;
        }
        else if (distanceRectLeft < distanceRectRight) {return Selected.LEFT;}
        return Selected.RIGHT;
    }
    protected double getColourDistance(Mat input,Rect rect, boolean left){
    submat= input.submat(rect);
    Scalar color= Core.mean(submat);
    if(left){
        hueLeft = color.val[0];
    } else{
        hueRight = color.val[0];
    }
    double distance;
    if(red){
        if (color.val[0] > 90){
            distance = 180 - color.val[0];
        }
        else{
            distance = color.val[0];
        }
    }
    else{
        if (color.val[0] > 120){
            distance = color.val[0] - 120;
        }
        else{
            distance = 120 - color.val[0];
        }
    }
    return distance;
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext){
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        selection = (Selected)userContext;
        switch (selection){
            case LEFT:
                canvas.drawRect(drawRectangleLeft,selectedPaint);
                canvas.drawRect(drawRectangleRight,nonSelectedPaint);
            case RIGHT:
                canvas.drawRect(drawRectangleLeft,nonSelectedPaint);
                canvas.drawRect(drawRectangleRight,selectedPaint);
            case NONE:
                canvas.drawRect(drawRectangleLeft,nonSelectedPaint);
                canvas.drawRect(drawRectangleRight,nonSelectedPaint);
        }
    }
    public Selected getSelection() {
        return selection;
    }
    public enum Selected {
        NONE,
        LEFT,
        RIGHT
    }
}
