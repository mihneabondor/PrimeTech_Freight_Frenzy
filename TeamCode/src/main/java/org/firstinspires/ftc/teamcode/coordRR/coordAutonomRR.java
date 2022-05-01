package org.firstinspires.ftc.teamcode.coordRR;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.camDetection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.security.ProtectionDomain;
import java.util.Locale;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.A_hardwareMap.COUNTS_PER_MM;
import static org.firstinspires.ftc.teamcode.A_hardwareMap.DriveValue;
import static org.firstinspires.ftc.teamcode.A_hardwareMap.StrafeValue;
import static org.firstinspires.ftc.teamcode.A_hardwareMap.TURN_SPEED;
import static org.firstinspires.ftc.teamcode.A_hardwareMap.TurnValue;


@Autonomous(name="Red right", group = "coordonate")
public class coordAutonomRR extends LinearOpMode  {
    FtcDashboard dashboard;
    private SampleMecanumDrive robot = null;
    camDetection pipeline;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);

        robot.setPoseEstimate(new Pose2d(11.6,-62));
        dashboard = FtcDashboard.getInstance();
        robot.pivotBrat.setPosition(0.5);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new camDetection(telemetry);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                                             dashboard.startCameraStream(webcam, 120);
                                         }

                                         @Override
                                         public void onError(int errorCode) {

                                         }
                                     }

        );
        waitForStart();

        telemetry.addData("unghi", robot.getRawExternalHeading());

        switch (pipeline.getCaz()) {
            case UNU:
                caz1();
                break;
            case DOI:
                caz2();
                break;
            case TREI:
                caz3();
                break;
            default:
                caz3();
                break;
        }
        webcam.stopStreaming();
    }


    public TrajectorySequence mergeInWarehouse = robot.trajectorySequenceBuilder(new Pose2d(-30, -35, Math.toRadians(90)))
            .addTemporalMarker(0.5, () -> {
                robot.ridicare.setPower(-0.8);
            })
            .addTemporalMarker(1.18, () -> {
                robot.ridicare.setPower(0);
            })
            .lineToLinearHeading(new Pose2d(18, -80, Math.toRadians(0)))
            .forward(60)
            .addDisplacementMarker(() -> {
                robot.gheara.setPosition(0.2);
            })
            .lineTo(new Vector2d(15, -82))
            .addTemporalMarker(7.2, () -> {
                robot.ridicare.setPower(0.8);
            })
            .addTemporalMarker(8.7, () -> {
                robot.ridicare.setPower(0);
            })
            .lineToLinearHeading(new Pose2d(-25, -40, Math.toRadians(84)))
            .addTemporalMarker(9.5, () -> {
                robot.gheara.setPosition(0.5);
            })
            .build();

    public TrajectorySequence parcareInWarehouse = robot.trajectorySequenceBuilder(new Pose2d(-30, -35, Math.toRadians(90)))
            .lineToLinearHeading(new Pose2d(18, -80, Math.toRadians(0)))
            .addTemporalMarker(0.5, () -> {
                robot.ridicare.setPower(-0.8);
            })
            .addTemporalMarker(1.35, () -> {
                robot.ridicare.setPower(0);
            })
            .forward(40)
            .build();

    private void caz3()
    {
        TrajectorySequence punePreload = robot.trajectorySequenceBuilder(new Pose2d(11.6, -67, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    robot.gheara.setPosition(0.26);
                })
                .lineTo(new Vector2d(-30, -35))
                .addTemporalMarker(0, () -> {
                    robot.ridicare.setPower(0.8);
                })
                .addTemporalMarker(1.3, () -> {
                    robot.ridicare.setPower(0);
                })
                .addTemporalMarker(1.7, () -> {
                    robot.gheara.setPosition(0.5);
                })
                .build();

        robot.followTrajectorySequence(punePreload);

        robot.setExternalHeading(Math.toRadians(90));

        robot.followTrajectorySequence(mergeInWarehouse);
        robot.followTrajectorySequence(parcareInWarehouse);
    }

    private void caz2() {
        TrajectorySequence punePreload = robot.trajectorySequenceBuilder(new Pose2d(11.6, -67, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    robot.gheara.setPosition(0.26);
                })
                .lineTo(new Vector2d(-30, -35))
                .addTemporalMarker(0, () -> {
                    robot.ridicare.setPower(0.8);
                })
                .addTemporalMarker(0.7, () -> {
                    robot.ridicare.setPower(0);
               })
                .addTemporalMarker(1.7, () -> {
                    robot.gheara.setPosition(0.5);
                })
                .build();

        robot.followTrajectorySequence(punePreload);

        robot.setExternalHeading(Math.toRadians(90));

        robot.followTrajectorySequence(mergeInWarehouse);
        robot.followTrajectorySequence(parcareInWarehouse);
    }

    private void caz1() {
        TrajectorySequence punePreload = robot.trajectorySequenceBuilder(new Pose2d(11.6, -67, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    robot.gheara.setPosition(0.26);
                })
                .lineTo(new Vector2d(-30, -35))
                .addTemporalMarker(0, () -> {
                    robot.ridicare.setPower(0.5);
                })
                .addTemporalMarker(0.6, () -> {
                    robot.ridicare.setPower(0);
                })
                .addTemporalMarker(1.7, () -> {
                    robot.gheara.setPosition(0.5);
                })
                .build();

        robot.followTrajectorySequence(punePreload);

        robot.setExternalHeading(Math.toRadians(90));

        robot.followTrajectorySequence(mergeInWarehouse);
        robot.followTrajectorySequence(parcareInWarehouse);
    }

}
