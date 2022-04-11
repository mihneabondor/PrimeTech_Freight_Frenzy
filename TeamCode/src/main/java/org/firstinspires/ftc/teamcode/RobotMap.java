package org.firstinspires.ftc.teamcode;

import android.media.AudioRecord;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotMap {
    Orientation lastAngles = new Orientation();
    double globalAngle;
    public DcMotor stangaFata;
    public DcMotor dreaptaFata;
    public DcMotor stangaSpate;
    public DcMotor dreaptaSpate;
    ElapsedTime runtime = new ElapsedTime();
    private LinearOpMode opMode;
    public BNO055IMU imu;

    public RobotMap(HardwareMap hardwareMap, LinearOpMode opMode) {

        stangaFata=hardwareMap.get(DcMotor.class, "stangaFata");
        dreaptaFata=hardwareMap.get(DcMotor.class, "dreaptaFata");
        stangaSpate=hardwareMap.get(DcMotor.class,"stangaSpate");
        dreaptaSpate=hardwareMap.get(DcMotor.class, "dreaptaSpate");
        this.opMode = opMode;


        stangaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stangaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        while (imu.isGyroCalibrated());



    }



    public void runUsingEncoders(double p, int ticks, double timeout) {

        runtime.reset();

        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stangaFata.setTargetPosition(-ticks);
        stangaSpate.setTargetPosition(-ticks);
        dreaptaFata.setTargetPosition(ticks);
        dreaptaSpate.setTargetPosition(ticks);

        stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (dreaptaFata.isBusy() && dreaptaSpate.isBusy() && stangaFata.isBusy() && stangaSpate.isBusy()
                && runtime.seconds() < timeout && opMode.opModeIsActive()){
            stangaSpate.setPower(p);
            stangaFata.setPower(p);
            dreaptaSpate.setPower(p);
            dreaptaFata.setPower(p);
        }

        stopDriving();
    }

    public void strafe(double p, int ticks, double timeout) {

        runtime.reset();

        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stangaFata.setTargetPosition(-ticks);
        stangaSpate.setTargetPosition(ticks);
        dreaptaFata.setTargetPosition(-ticks);
        dreaptaSpate.setTargetPosition(ticks);

        stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (dreaptaFata.isBusy() && dreaptaSpate.isBusy() && stangaFata.isBusy() && stangaSpate.isBusy()
                && runtime.seconds() < timeout && opMode.opModeIsActive()){
            stangaSpate.setPower(p);
            stangaFata.setPower(p);
            dreaptaSpate.setPower(p);
            dreaptaFata.setPower(p);
        }

        stopDriving();
    }

    public void strafediagonalaprincipala(double p, int ticks, double timeout) {

        runtime.reset();

        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stangaFata.setTargetPosition(-ticks);
        stangaSpate.setTargetPosition(-ticks);
        dreaptaFata.setTargetPosition(ticks);
        dreaptaSpate.setTargetPosition(ticks);

        stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (dreaptaFata.isBusy() && dreaptaSpate.isBusy() && stangaFata.isBusy() && stangaSpate.isBusy()
                && runtime.seconds() < timeout && opMode.opModeIsActive()){
            stangaSpate.setPower(p);
            stangaFata.setPower(0);
            dreaptaSpate.setPower(0);
            dreaptaFata.setPower(p);
        }

        stopDriving();
    }
    public void strafediagonalasecundara(double p, int ticks, double timeout) {

        runtime.reset();

        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        stangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stangaFata.setTargetPosition(-ticks);
        stangaSpate.setTargetPosition(ticks);
        dreaptaFata.setTargetPosition(-ticks);
        dreaptaSpate.setTargetPosition(ticks);

        stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (dreaptaFata.isBusy() && dreaptaSpate.isBusy() && stangaFata.isBusy() && stangaSpate.isBusy()
                && runtime.seconds() < timeout && opMode.opModeIsActive()){
            stangaSpate.setPower(0);
            stangaFata.setPower(p);
            dreaptaSpate.setPower(p);
            dreaptaFata.setPower(0);
        }

        stopDriving();
    }

    public int convertire(double cm){
        int ticks=751;
        int cmperrotatie=30;
        int x = (int)(cm*ticks)/cmperrotatie;
        return x;
    }

    public double SQRT(double x) {
        if (x < 0) return -Math.sqrt(Math.abs(x));
        return Math.sqrt(x);
    }

    public void stopDriving() {
        stangaFata.setPower(0);
        stangaSpate.setPower(0);
        dreaptaFata.setPower(0);
        dreaptaSpate.setPower(0);
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if(deltaAngle < -180)
            deltaAngle += 360;
        else if(deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle +=deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    public void rotate(int degrees, double power, int timeout) {
        stangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime.reset();

        resetAngle();

        if (degrees < 0) {
            power = -power;
        }

        stangaSpate.setPower(power);
        stangaFata.setPower(power);
        dreaptaSpate.setPower(power);
        dreaptaFata.setPower(power);

        if (degrees < 0) {
            while (opMode.opModeIsActive() && getAngle() > degrees && runtime.seconds() < timeout) {}
        }
        else {
            while (opMode.opModeIsActive() && getAngle() < degrees && runtime.seconds() < timeout);
        }

        stopDriving();
    }

}