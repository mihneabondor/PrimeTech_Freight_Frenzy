package org.firstinspires.ftc.teamcode.coordRR;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class camRR extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Caz {
        UNU,
        DOI,
        TREI
    }
    private volatile Caz caz;

    static final Rect LEFT_ROI = new Rect (
            new Point(0, 90),
            new Point(45, 170)
    );

    static final Rect MID_ROI = new Rect(
            new Point(75, 90),
            new Point(135, 170)
    );
    static final Rect RIGHT_ROI = new Rect(
            new Point(170, 90),
            new Point(230, 170)
    );
    static double PERCENT_TRESHOLD = 0.2;
    public camRR(Telemetry t) {telemetry = t;}

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // (hMin = 19 , sMin = 0, vMin = 196), (hMax = 179 , sMax = 51, vMax = 255)
        // verde: Scalar lowHSV = new Scalar(59, 50, 40);
        //        Scalar highHSV = new Scalar(99, 88, 100);
        // (hMin = 41 , sMin = 42, vMin = 0), (hMax = 95 , sMax = 255, vMax = 245)
        // (hMin = 0 , sMin = 0, vMin = 0), (hMax = 179 , sMax = 209, vMax = 62)
        Scalar lowHSV = new Scalar(0, 0, 0);
        Scalar highHSV = new Scalar(179, 209, 62);
        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat left = mat.submat(LEFT_ROI);
        Mat mid = mat.submat(MID_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftVal = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double midVal = Core.sumElems(mid).val[0] / MID_ROI.area() / 255;
        double rightVal = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        mid.release();
        right.release();

        telemetry.addData("left val:", (int) Core.sumElems(left).val[0]);
        telemetry.addData("left %:", Math.round(leftVal * 100) + "%");
        telemetry.addData("mid val:", (int) Core.sumElems(mid).val[0]);
        telemetry.addData("Right val:", (int) Core.sumElems(right).val[0]);
        telemetry.addData("mid %:", Math.round(midVal * 100) + "%");
        telemetry.addData("Right %:", Math.round(rightVal * 100) + "%");

        boolean rataLeft = leftVal > PERCENT_TRESHOLD;
        boolean rataMid = midVal > PERCENT_TRESHOLD;
        boolean rataRight = rightVal > PERCENT_TRESHOLD;

        if(rataRight) {
            caz = Caz.TREI;
            telemetry.addData("Caz", 3);
        } else if (rataMid) {
            caz = Caz.DOI;
            telemetry.addData("Caz", 2);
        } else if(rataLeft){
            caz = Caz.UNU;
            telemetry.addData("Caz", 1);
        }
        telemetry.update();
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);

        Scalar colorNimic = new Scalar(255, 0, 0);
        Scalar colorRata = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, caz == caz.UNU ? colorRata: colorNimic);
        Imgproc.rectangle(mat, MID_ROI, caz == caz.DOI ? colorRata: colorNimic);
        Imgproc.rectangle(mat, RIGHT_ROI, caz == caz.TREI ? colorRata: colorNimic);

        return mat;
    }

    public Caz getCaz() {
        return caz;
    }
}
