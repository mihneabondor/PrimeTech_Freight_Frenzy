package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.Locale;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class   A_hardwareMap
{
    public static final double      PRINDERE_INITIAL =  0.25 ;
    public static final double      PRINDERE_COMPLETA = 1;
    public static final double      TAVA_JOS =  0.75 ;
    public static final double      TAVA_SUS   = 0.25 ;
    public static final double      DriveValue = 2.43;
    public static final double      TurnValue = 2;
    public static final double      StrafeValue = 2;
    public static final double      PullValue = 125;
    public static final double      COUNTS_PER_MOTOR_REV = 1120 ;    
    public static final double      DRIVE_GEAR_REDUCTION = 2.0 ;    
    public static final double      WHEEL_DIAMETER_MM = 4.0 * 25.4;     
    public static final double     COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    public static final double      DRIVE_SPEED = 1;
    public static final double      TURN_SPEED = 0.5;
    public static final double      PULL_SPEED = 0.1;

    public DcMotor RightFrontMotor = null;
    public DcMotor RightBackMotor = null;
    public DcMotor LeftFrontMotor = null;
    public DcMotor LeftBackMotor = null;

    public DcMotor ridicare = null;
    public DcMotor carusel = null;

    public Servo rotire = null;
    public Servo prindere = null;

    public DistanceSensor distanta;

    HardwareMap A_hardwareMap =  null;

    public void init(HardwareMap ahwMap) {
        A_hardwareMap = ahwMap;


        RightBackMotor = A_hardwareMap.get(DcMotor.class, "RightBackMotor");
        RightFrontMotor = A_hardwareMap.get(DcMotor.class, "RightFrontMotor");
        LeftFrontMotor = A_hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        LeftBackMotor = A_hardwareMap.get(DcMotor.class, "LeftBackMotor");

        carusel = A_hardwareMap.get(DcMotor.class, "carusel");
        ridicare = A_hardwareMap.get(DcMotor.class, "ridicare");

        rotire = A_hardwareMap.get(Servo.class, "rotire");
        prindere = A_hardwareMap.get(Servo.class, "prindere");

        distanta =  A_hardwareMap.get(DistanceSensor.class, "distanta");
        
        RightBackMotor.setDirection(FORWARD);
        RightFrontMotor.setDirection(FORWARD);
        LeftBackMotor.setDirection(FORWARD);
        LeftFrontMotor.setDirection(FORWARD);
        
        /*
        RightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
        
        RightBackMotor.setPower(0);
        RightFrontMotor.setPower(0);
        LeftFrontMotor.setPower(0);
        LeftBackMotor.setPower(0);

        rotire.setPosition(0.5);
    }




}