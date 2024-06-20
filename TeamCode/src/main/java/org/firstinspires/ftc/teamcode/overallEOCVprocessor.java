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

import java.util.Arrays;
import java.util.List;


public class overallEOCVprocessor implements VisionProcessor{
    public Rect rectLeft;
    public  Rect rectRight;
    public boolean red;
    public double percentageLeft;
    public double percentageRight;
    Selected selection = Selected.NONE;
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
        // mask & concatenate
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat ranged = new Mat();
        Core.inRange(hsvMat, new Scalar(0, 70, 50), new Scalar(10, 255, 255), mask2);
        Core.inRange(hsvMat, new Scalar(170, 70, 50), new Scalar(180, 255, 255), mask1);
        List<Mat> src = Arrays.asList(mask1, mask2);
        Core.hconcat(src, ranged);
        // submat & count
        percentageLeft = Core.countNonZero(ranged.submat(rectLeft)) / (ranged.submat(rectLeft).total()/3.0);
        percentageRight = Core.countNonZero(ranged.submat(rectRight)) / (ranged.submat(rectRight).total()/3.0);


        if (Math.abs(percentageLeft - percentageRight) < 0.05){
            return Selected.NONE;
        }
        if (percentageLeft > percentageRight) {return Selected.LEFT;}
        return Selected.RIGHT;
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
