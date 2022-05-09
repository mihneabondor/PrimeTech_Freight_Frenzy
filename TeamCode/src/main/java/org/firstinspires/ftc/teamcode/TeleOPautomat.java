package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.A_hardwareMap;

@TeleOp(name = "Teleop automat")
public class TeleOPautomat extends OpMode {
    private DcMotor stangaFata;
    private DcMotor dreaptaFata;
    private DcMotor stangaSpate;
    private DcMotor dreaptaSpate;
    private DcMotor ridicare;
    private double n;
    public Servo pivotBrat;

    A_hardwareMap robot = new A_hardwareMap();

    @Override
    public void init() {
        robot.init(hardwareMap);

        double px =n* -gamepad1.left_stick_x;
        double py =n* gamepad1.left_stick_y;
        double pp =n* gamepad1.right_stick_x;

        stangaSpate.setPower(py-px-pp);
        stangaFata.setPower(py+px-pp);
        dreaptaSpate.setPower(-py-px-pp);
        dreaptaFata.setPower(-py+px-pp);

        pivotBrat = hardwareMap.servo.get("rotire");
        pivotBrat.setPosition(0.5);
        stangaFata=hardwareMap.dcMotor.get("LeftFrontMotor");
        dreaptaFata=hardwareMap.dcMotor.get("RightFrontMotor");
        stangaSpate=hardwareMap.dcMotor.get("LeftBackMotor");
        dreaptaSpate=hardwareMap.dcMotor.get("RightBackMotor");
        ridicare = hardwareMap.dcMotor.get("ridicare");

        ridicare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if(gamepad2.a) {

            ridicare.setTargetPosition(3000);
            ridicare.setPower(0.5);
        }

        if(gamepad2.b) {
            ridicare.setPower(0);
            ridicare.setPower(0.5);
        }

        telemetry.addData("Ridicare encoder", ridicare.getCurrentPosition());
    }
}
