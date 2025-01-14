package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.A_hardwareMap.COUNTS_PER_MM;
import static org.firstinspires.ftc.teamcode.A_hardwareMap.DriveValue;
import static org.firstinspires.ftc.teamcode.A_hardwareMap.StrafeValue;
import static org.firstinspires.ftc.teamcode.A_hardwareMap.TURN_SPEED;
import static org.firstinspires.ftc.teamcode.A_hardwareMap.TurnValue;

@Autonomous(name="Albastru Dreapta", group = "oficial")

public class AutonomAlbastruDreapta extends LinearOpMode{
    A_hardwareMap robot   = new A_hardwareMap();

    public double putere = 1;

    private ElapsedTime runtime = new ElapsedTime();
    public int caz = 1;

    @Override
    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.LeftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LeftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.ridicare.setDirection(FORWARD);
        
        robot.RightBackMotor.setDirection(REVERSE);
        robot.RightFrontMotor.setDirection(REVERSE);
        robot.LeftBackMotor.setDirection(FORWARD);
        robot.LeftFrontMotor.setDirection(FORWARD);
        telemetry.update();


        waitForStart();
        
        ///sleep(2000);
        robot.prindere.setPosition(0.2);
        sleep(250);
        robot.ridicare.setPower(0.7);
        sleep(550);
        robot.ridicare.setPower(0);
        StopAllMotion();
        DriveForward(15, 0.5, 0.5, 0.5, 0.5);
        StopAllMotion();
        /*
        StrafeRight(50,0.5,0.5,0.5,0.5);
        StopAllMotion();
        DriveForward(40,0.5,0.5,0.5);
        StopAllMotion();
        */
        
        if(robot.distanta.getDistance(DistanceUnit.CM) <= 30) {
            
            StrafeRight(115, 0.7);
            sleep(1000);
            robot.ridicare.setPower(0.4);
            sleep(400);
            StopAllMotion();
            DriveForward(34, 0.6, 0.6, 0.6, 0.6);
            StopAllMotion();
            sleep(500);
            robot.prindere.setPosition(0.5);
            StopAllMotion();
            robot.ridicare.setPower(-0.1);
            sleep(3);
            DriveBackward(37, 0.6, 0.6, 0.6, 0.6);
            StopAllMotion();
            robot.ridicare.setPower(0);
            StopAllMotion();
            RotateLeft(82);
            StopAllMotion();
            DriveBackward(180, 0.5, 0.5, 0.5, 0.5);
            StopAllMotion();
            StrafeRight(50 , 0.5);
            sleep(1000);
            robot.carusel.setPower(-1);
            sleep(2000);
            robot.ridicare.setPower(-0.1);
            StrafeLeft(100, 0.5);
            DriveBackward(50 , 0.5 , 0.5 , 0.5 , 0.5);
            
        }
        
        else
        {
            StrafeRight(40, 0.4);
            if (robot.distanta.getDistance(DistanceUnit.CM) <= 30)
            {
                StrafeRight(85, 0.7);
                sleep(1000);
                robot.ridicare.setPower(-0.1);
                sleep(400);
                StopAllMotion();
                DriveForward(30, 0.6, 0.6, 0.6, 0.6);
                StopAllMotion();
                sleep(500);
                robot.prindere.setPosition(0.5);
                StopAllMotion();
                robot.ridicare.setPower(-0.1);
                sleep(3);
                DriveBackward(37, 0.6, 0.6, 0.6, 0.6);
                StopAllMotion();
                robot.ridicare.setPower(0);
                StopAllMotion();
                RotateLeft(82);
                StopAllMotion();
                DriveBackward(190, 0.5, 0.5, 0.5, 0.5);
                StopAllMotion();
                StrafeRight(65 , 0.5);
                sleep(1000);
                robot.carusel.setPower(-1);
                sleep(2000);
                robot.ridicare.setPower(-0.1);
                StrafeLeft(105, 0.5);
                DriveBackward(45 , 0.5 , 0.5 , 0.5 , 0.5);
            }
            else
            if (robot.distanta.getDistance(DistanceUnit.CM) > 30)
            {
                StrafeRight(70, 0.7);
                sleep(1000);
                robot.ridicare.setPower(0.6);
                sleep(750);
                StopAllMotion();
                DriveForward(30, 0.6, 0.6, 0.6, 0.6);
                StopAllMotion();
                sleep(500);
                robot.prindere.setPosition(0.5);
                StopAllMotion();
                robot.ridicare.setPower(-0.1);
                sleep(3);
                DriveBackward(37, 0.6, 0.6, 0.6, 0.6);
                StopAllMotion();
                robot.ridicare.setPower(0);
                StopAllMotion();
                RotateLeft(82);
                StopAllMotion();
                DriveBackward(190, 0.5, 0.5, 0.5, 0.5);
                StopAllMotion();
                StrafeRight(65 , 0.5);
                sleep(1000);
                robot.carusel.setPower(-1);
                sleep(2000);
                robot.ridicare.setPower(-0.2);
                StrafeLeft(105, 0.5);
                DriveBackward(45 , 0.5 , 0.5 , 0.5 , 0.5);
            }
        }
        
        telemetry.update();
    }



    public void EncoderDrive (double speed_RF,double speed_RB,double speed_LF,double speed_LB, double distance, double timeoutS)
    {
        if (opModeIsActive()) {
            int newBackLeftTarget;
            int newBackRightTarget;
            int newFrontLeftTarget;
            int newFrontRightTarget;
            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newBackLeftTarget = robot.LeftBackMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DriveValue);
                newBackRightTarget = robot.RightBackMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DriveValue);
                newFrontLeftTarget = robot.LeftFrontMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DriveValue);
                newFrontRightTarget = robot.RightFrontMotor.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DriveValue);

                robot.LeftBackMotor.setTargetPosition(newBackLeftTarget);
                robot.RightBackMotor.setTargetPosition(newBackRightTarget);
                robot.LeftFrontMotor.setTargetPosition(newFrontLeftTarget);
                robot.RightFrontMotor.setTargetPosition(newFrontRightTarget);

                // Turn On RUN_TO_POSITION
                robot.LeftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.RightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.LeftBackMotor.setPower(speed_LB);
                robot.RightBackMotor.setPower(speed_RB);
                robot.LeftFrontMotor.setPower(speed_LF);
                robot.RightFrontMotor.setPower(speed_RF);

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (
                                robot.LeftBackMotor.isBusy() && robot.RightBackMotor.isBusy() &&
                                        robot.LeftFrontMotor.isBusy() && robot.RightFrontMotor.isBusy()
                        )) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d"
                            , newFrontLeftTarget, newFrontRightTarget
                            , newBackLeftTarget, newBackRightTarget
                    );
                    telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            robot.LeftFrontMotor.getCurrentPosition(),
                            robot.RightFrontMotor.getCurrentPosition()
                            ,
                            robot.LeftBackMotor.getCurrentPosition(),
                            robot.RightBackMotor.getCurrentPosition()
                    );
                    telemetry.addData("RF", String.valueOf(robot.RightFrontMotor.getPower()));
                    telemetry.addData("RB", String.valueOf(robot.RightBackMotor.getPower()));
                    telemetry.addData("LF", String.valueOf(robot.LeftFrontMotor.getPower()));
                    telemetry.addData("LB", String.valueOf(robot.LeftBackMotor.getPower()));
                    telemetry.update();
                }

                // Stop all motion;


                //robot.LeftBackMotor.setPower(0);
                //robot.RightBackMotor.setPower(0);
                //robot.LeftFrontMotor.setPower(0);
                //robot.RightFrontMotor.setPower(0);


                /* COMMENT THESE FOR SPEED */

                // Turn off RUN_TO_POSITION
                robot.LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //sleep(250);   // optional pause after each move
            }
        }
    }
    public void EncoderStrafe(double speed, double distance, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        if (opModeIsActive()) {

            newBackLeftTarget = robot.LeftBackMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * StrafeValue);
            newBackRightTarget = robot.RightBackMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * StrafeValue);
            newFrontLeftTarget = robot.LeftFrontMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * StrafeValue);
            newFrontRightTarget = robot.RightFrontMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * StrafeValue);

            robot.LeftBackMotor.setTargetPosition(newBackLeftTarget);
            robot.RightBackMotor.setTargetPosition(newBackRightTarget);
            robot.LeftFrontMotor.setTargetPosition(newFrontLeftTarget);
            robot.RightFrontMotor.setTargetPosition(newFrontRightTarget);

            robot.LeftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.LeftBackMotor.setPower(-speed);
            robot.RightBackMotor.setPower(-speed);
            robot.LeftFrontMotor.setPower(speed);
            robot.RightFrontMotor.setPower(speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (
                            robot.LeftBackMotor.isBusy() && robot.RightBackMotor.isBusy() &&
                                    robot.LeftFrontMotor.isBusy() && robot.RightFrontMotor.isBusy()
                    ))
            {

                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d"
                        , newFrontLeftTarget, newFrontRightTarget
                        , newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.LeftFrontMotor.getCurrentPosition(),
                        robot.RightFrontMotor.getCurrentPosition()
                        ,
                        robot.LeftBackMotor.getCurrentPosition(),
                        robot.RightBackMotor.getCurrentPosition());

                telemetry.update();
            }

            robot.LeftBackMotor.setPower(0);
            robot.RightBackMotor.setPower(0);
            robot.LeftFrontMotor.setPower(0);
            robot.RightFrontMotor.setPower(0);

            robot.LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void DriveBackward (double distance, double speed_RF,double speed_RB,double speed_LF,double speed_LB)
    {
        EncoderDrive(speed_RF,speed_RB,speed_LF,speed_LB, distance,15);
        //sleep(100);
    }

    public void DriveForward (double distance, double speed_RF,double speed_RB,double speed_LF,double speed_LB)
    {
        EncoderDrive(speed_RF,speed_RB,speed_LF,speed_LB, -distance,10);
        //sleep(100);
    }

    public void StrafeRight(double distance, double speed)
    {
        EncoderStrafe(-speed, -distance, 15);
        sleep(100);
    }

    public void StrafeLeft (double distance, double speed)
    {
        EncoderStrafe(speed, distance, 15);
        sleep(100);
    }

    public void EncoderTurn(double speed, double distance, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        if (opModeIsActive()) {

            newBackLeftTarget = robot.LeftBackMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * TurnValue);
            newBackRightTarget = robot.RightBackMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * TurnValue);
            newFrontLeftTarget = robot.LeftFrontMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * TurnValue);
            newFrontRightTarget = robot.RightFrontMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * TurnValue);

            robot.LeftBackMotor.setTargetPosition(newBackLeftTarget);
            robot.RightBackMotor.setTargetPosition(newBackRightTarget);
            robot.LeftFrontMotor.setTargetPosition(newFrontLeftTarget);
            robot.RightFrontMotor.setTargetPosition(newFrontRightTarget);

            robot.LeftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.LeftBackMotor.setPower(speed);
            robot.RightBackMotor.setPower(speed);
            robot.LeftFrontMotor.setPower(speed);
            robot.RightFrontMotor.setPower(speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (
                            robot.LeftBackMotor.isBusy() && robot.RightBackMotor.isBusy() &&
                                    robot.LeftFrontMotor.isBusy() && robot.RightFrontMotor.isBusy()
                    ))
            {
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d"
                        , newFrontLeftTarget, newFrontRightTarget
                        , newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.LeftFrontMotor.getCurrentPosition(),
                        robot.RightFrontMotor.getCurrentPosition()
                        ,
                        robot.LeftBackMotor.getCurrentPosition(),
                        robot.RightBackMotor.getCurrentPosition());
                telemetry.update();
            }

            //robot.LeftBackMotor.setPower(0);
            //robot.RightBackMotor.setPower(0);
            //robot.LeftFrontMotor.setPower(0);
            //robot.RightFrontMotor.setPower(0);

            robot.LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void RotateRight(double angle)
    {
        EncoderTurn(TURN_SPEED, angle, 15);
    }

    public void RotateLeft(double angle)
    {
        EncoderTurn(-TURN_SPEED, -angle, 15);
    }

    public void StopAllMotion() {
        robot.LeftFrontMotor.setPower(0);
        robot.RightFrontMotor.setPower(0);
        robot.LeftBackMotor.setPower(0);
        robot.RightBackMotor.setPower(0);

    }

    public void runUsingEncoders(double p, int ticks, double timeout) {

        runtime.reset();

        robot.LeftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LeftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LeftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.LeftFrontMotor.setTargetPosition(-ticks);
        robot.LeftBackMotor.setTargetPosition(-ticks);
        robot.RightFrontMotor.setTargetPosition(-ticks);
        robot.RightBackMotor.setTargetPosition(-ticks);

        robot.LeftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LeftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.RightFrontMotor.isBusy() && robot.RightBackMotor.isBusy() && robot.LeftFrontMotor.isBusy() && robot.LeftBackMotor.isBusy()
                && runtime.seconds() < timeout){
            robot.LeftFrontMotor.setPower(p);
            robot.LeftBackMotor.setPower(p);
            robot.RightFrontMotor.setPower(p);
            robot.RightBackMotor.setPower(p);
        }

        StopAllMotion();
    }

    
    public int convertire(double cm){
        int ticks = 563;
        int cmperrotatie=30;
        int x = (int)(cm*ticks)/cmperrotatie;
        return x;
    }

}

