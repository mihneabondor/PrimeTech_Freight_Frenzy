package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Test Camera", group = "Camera")
public class OPTestCamera extends LinearOpMode {
    OpenCvCamera webcam;
    FtcDashboard dashboard;
    camDetection detector;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        detector = new camDetection(telemetry);
        webcam.setPipeline(detector);
        dashboard = FtcDashboard.getInstance();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                                             dashboard.startCameraStream(webcam, 120);
                                         }
//                                         @Override
                                         public void onError(int errorCode) {
                                         }
                                     }

        );

        waitForStart(); // TODO: fix the fucking colors
        switch (detector.getCaz()) {
            case UNU:
                /// ...
                break;
            case DOI:
                break;
            case TREI:
                break;
        }
        webcam.stopStreaming();
    }
}
