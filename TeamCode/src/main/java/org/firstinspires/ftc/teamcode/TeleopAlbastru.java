package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



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

@TeleOp(name="Teleop Albastru", group="Iterative Opmode")

public class TeleopAlbastru extends OpMode
{
        
    private DcMotor stangaFata;
    private DcMotor dreaptaFata;
    private DcMotor stangaSpate;
    private DcMotor dreaptaSpate;
    public Servo pivotBrat;
    
    A_hardwareMap robot = new A_hardwareMap();
    
    private double rataStanga = 0;
    private double rataDreapta = 0;
    
    public  double r = 0.5;
    
    public double n=1;

    @Override
    public void init() {
        telemetry.addData("Status", "Before Initialization");

        robot.init(hardwareMap);
        
        pivotBrat = hardwareMap.servo.get("rotire");
        pivotBrat.setPosition(0.5);
        stangaFata=hardwareMap.dcMotor.get("LeftFrontMotor");
        dreaptaFata=hardwareMap.dcMotor.get("RightFrontMotor");
        stangaSpate=hardwareMap.dcMotor.get("LeftBackMotor");
        dreaptaSpate=hardwareMap.dcMotor.get("RightBackMotor");

        dreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dreaptaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stangaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stangaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop()
    {
            double px =n* -gamepad1.left_stick_x;
            double py =n* gamepad1.left_stick_y;
            double pp =n* gamepad1.right_stick_x;

            stangaSpate.setPower(py-px-pp);
            stangaFata.setPower(py+px-pp);
            dreaptaSpate.setPower(-py-px-pp);
            dreaptaFata.setPower(-py+px-pp);
            
            
        ///robot.carusel.setPower(1.5 * gamepad1.left_trigger);
        ///robot.carusel.setPower(-1.5 * gamepad1.right_trigger);
        
        robot.ridicare.setPower(0.9 * gamepad2.left_stick_y);
        
        
        if(gamepad2.left_trigger >= 0.5)
            pivotBrat.setPosition(pivotBrat.getPosition()+0.01);
        if(gamepad2.right_trigger >= 0.5)
            pivotBrat.setPosition(pivotBrat.getPosition()-0.01);
        
        if(gamepad2.dpad_down)
            robot.prindere.setPosition(0.5);
        if(gamepad2.dpad_up)
            robot.prindere.setPosition(0.26);
    
        if(gamepad2.b)
            pivotBrat.setPosition(0.5);
        
        if(gamepad1.x)
            n=0.6;
        if(gamepad1.a)
            n=0.3;
        if(gamepad1.b)
            n=1;
        
                
        if(gamepad2.x && pivotBrat.getPosition() <= 0.83)
            pivotBrat.setPosition(pivotBrat.getPosition()+0.01);
        
        rataDreapta = gamepad1.left_trigger;
        
        if(rataDreapta > 0) robot.carusel.setPower(-rataDreapta);
        else {
            robot.carusel.setPower(0);
            robot.carusel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        
        telemetry.addData("rotire:",r);
        telemetry.addData("servo rotire:", pivotBrat.getPosition());
        telemetry.addData("distanta:", robot.distanta.getDistance(DistanceUnit.CM));
        telemetry.addData("n:", n);
        telemetry.update();
        
    }
    
    /*
    public void loop() {
    if(System.currentTimeMillis() - setTime > 10000 && !hasRun) {
         //Will only run after 10 seconds, and will only run once
        hasRun = true;
        ///servo.setPosition(0);
    }
    */
}