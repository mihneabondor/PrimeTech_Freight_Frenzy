package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class camDetection extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Caz {
        UNU,
        DOI,
        TREI
    }
    private volatile Caz caz = Caz.UNU;
    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75)
    );
    static final Rect RIGHT_ROI = new Rect(
            new Point(220, 35),
            new Point(280, 75)
    );
    static double PERCENT_TRESHOLD = 0.4;
    public camDetection(Telemetry t) {telemetry = t;}

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // TODO: (hMin = 19 , sMin = 0, vMin = 196), (hMax = 179 , sMax = 51, vMax = 255)
        // verde: Scalar lowHSV = new Scalar(59, 50, 40);
        //        Scalar highHSV = new Scalar(99, 88, 100);
        Scalar lowHSV = new Scalar(19, 0, 196);
        Scalar highHSV = new Scalar(179, 51, 255);
        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftVal = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightVal = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left val:", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right val:", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left %:", Math.round(leftVal * 100) + "%");
        telemetry.addData("Right %:", Math.round(rightVal * 100) + "%");

        boolean rataLeft = leftVal > PERCENT_TRESHOLD;
        boolean rataRight = rightVal > PERCENT_TRESHOLD;

        if(rataRight) {
            caz = Caz.TREI;
            telemetry.addData("Caz", 3);
        } else if (rataLeft) {
            caz = Caz.UNU;
            telemetry.addData("Caz", 1);
        } else {
            caz = Caz.DOI;
            telemetry.addData("Caz", 2);
        }
        telemetry.update();
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);

        Scalar colorNimic = new Scalar(255, 0, 0); // LOW
        Scalar colorRata = new Scalar(0, 255, 0);  // HIGH

        Imgproc.rectangle(mat, LEFT_ROI, caz == caz.UNU ? colorRata: colorNimic);
        Imgproc.rectangle(mat, RIGHT_ROI, caz == caz.TREI ? colorRata: colorNimic);

        return mat;
    }

    public Caz getCaz() {
        return caz;
    }
}
