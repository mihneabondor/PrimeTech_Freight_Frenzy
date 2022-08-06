package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="SD Teleop", group="SD")

public class swervedDiffTeleOP extends OpMode
{

    private DcMotor dreaptaSpate;
    private DcMotor dreaptaFata;


    @Override
    public void init() {
        telemetry.addData("Status", "Before Initialization");

        dreaptaSpate= hardwareMap.dcMotor.get("RightBackMotor");
        dreaptaFata= hardwareMap.dcMotor.get("RightFrontMotor");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop()
    {
        double px =-gamepad1.left_stick_x;
        double py =gamepad1.left_stick_y;
        double pp =gamepad1.right_stick_x;

        dreaptaSpate.setPower(py-px-pp);
        dreaptaFata.setPower(py+px-pp);
        }
}