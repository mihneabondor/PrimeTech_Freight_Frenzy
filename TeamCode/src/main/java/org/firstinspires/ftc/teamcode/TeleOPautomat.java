package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.A_hardwareMap;

@TeleOp(name = "Teleop automat")
public class TeleOPautomat extends OpMode {
    A_hardwareMap robot = new A_hardwareMap();
    @Override
    public void init() {
        robot.ridicare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.ridicare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if(gamepad2.a) {
            robot.ridicare.setTargetPosition(3000);
            robot.ridicare.setPower(0.5);
        }

        if(gamepad2.b) {
            robot.ridicare.setPower(0);
            robot.ridicare.setPower(0.5);
        }

        telemetry.addData("Ridicare encoder", robot.ridicare.getCurrentPosition());
    }
}
