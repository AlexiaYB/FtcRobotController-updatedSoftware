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

public class DrawRectangleProcessor implements VisionProcessor {
    public Rect rectLeft = new Rect(70, 0, 100, 83);
    public Rect rectRight = new Rect(445, 0, 100, 150);
    Selected selection = Selected.NONE;
    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        double satRectRight = getAvgSaturation(hsvMat, rectRight);
        if (satRectLeft > satRectRight) {return Selected.LEFT;}
        return Selected.RIGHT;
    }
    protected double getAvgSaturation(Mat input,Rect rect){
    submat= input.submat(rect);
    Scalar color= Core.mean(submat);
    return color.val[1];
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
