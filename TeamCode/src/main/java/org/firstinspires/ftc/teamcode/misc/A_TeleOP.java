package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.teamcode.A_hardwareMap;

@Disabled
@TeleOp(name="A_TeleOP", group="Iterative Opmode")

public class A_TeleOP extends OpMode
{
    A_hardwareMap robot = new A_hardwareMap();
    
    public  double r = 0.5;
    
    public double n=1;

    @Override
    public void init() {
        telemetry.addData("Status", "Before Initialization");

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop()
    {
        
        double drive = n * gamepad1.right_stick_y;
        double turn = n * gamepad1.right_stick_x;

        double rightPower;
        double leftPower;

        rightPower = Range.clip(drive + turn,-1, 1);
        leftPower  = Range.clip(drive - turn,-1, 1);

        robot.RightFrontMotor.setPower(rightPower);
        robot.LeftFrontMotor.setPower(leftPower);
        robot.RightBackMotor.setPower(rightPower);
        robot.LeftBackMotor.setPower(leftPower);
        
        robot.carusel.setPower(gamepad2.right_trigger);
        
        robot.ridicare.setPower(0.7 * gamepad2.left_stick_y);
        
        
        if(gamepad2.dpad_left)
            r=r+0.005;
        if(gamepad2.dpad_right)
            r=r-0.005;
        robot.rotire.setPosition(r);
        
        if(gamepad2.dpad_up)
            robot.prindere.setPosition(0.26);
        if(gamepad2.dpad_down)
            robot.prindere.setPosition(0.5);
        
        if(gamepad1.x)
            n=1;
        if(gamepad1.a)
            n=0.50;
        
        
            
        telemetry.addData("rotire:",r);
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
    
    

