package org.firstinspires.ftc.teamcode.SwervedDifferential;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="SD Teleop", group="SD")

public class swervedDiffTeleOP extends OpMode
{

    private DcMotor dreaptaSpate;
    private DcMotor dreaptaFata;
    private DcMotor stangaSpate;
    private DcMotor stangaFata;

    int DEG_90_TARGET = 395;
    int DEG_0_TARGET = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Before Initialization");

        dreaptaSpate= hardwareMap.dcMotor.get("RightBackMotor");
        dreaptaFata= hardwareMap.dcMotor.get("RightFrontMotor");
        stangaFata = hardwareMap.dcMotor.get("LeftFrontMotor");
        stangaSpate = hardwareMap.dcMotor.get("LeftBackMotor");

        telemetry.addData("Status", "Initialized");

        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop()
    {
        double px =gamepad1.right_stick_x/2;
        double py;

        int pozDreaptaFata = dreaptaFata.getCurrentPosition();
        int pozDreaptaSpate = dreaptaSpate.getCurrentPosition();
        int difDreapta = (pozDreaptaFata - pozDreaptaSpate)/2;

        if(Math.abs(difDreapta)%1600 <= 1100 && Math.abs(difDreapta) >= 500) {
            py = -gamepad1.left_stick_y/2;
        } else {
            py = gamepad1.left_stick_y/2;
        }

        dreaptaSpate.setPower(py-px);
        dreaptaFata.setPower(py+px);
        stangaSpate.setPower(py-px);
        stangaFata.setPower(py+px);

        if(gamepad1.a) {
            dreaptaFata.setPower(0.8);
            dreaptaSpate.setPower(0.8);
            stangaFata.setPower(0.8);
            stangaSpate.setPower(0.8);

            setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(gamepad1.dpad_left) {
            setTargetPosition(DEG_90_TARGET);
        }

        if(gamepad1.dpad_up) {
            setTargetPosition(DEG_0_TARGET);
        }

        if(gamepad1.x) {
            setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        telemetry.addData("right front", dreaptaFata.getCurrentPosition());
        telemetry.addData("right back", dreaptaSpate.getCurrentPosition());
        telemetry.addData("left front", stangaFata.getCurrentPosition());
        telemetry.addData("left back", stangaSpate.getCurrentPosition());
        telemetry.addData("target position", dreaptaFata.getTargetPosition());
        telemetry.update();
        }

        private void setTargetPosition(int target) {
            dreaptaSpate.setTargetPosition(target);
            dreaptaFata.setTargetPosition(-target);

            stangaSpate.setTargetPosition(target);
            stangaFata.setTargetPosition(-target);
        }

        private void setEncoderMode(DcMotor.RunMode runMode) {
            dreaptaFata.setMode(runMode);
            dreaptaSpate.setMode(runMode);
            stangaFata.setMode(runMode);
            stangaSpate.setMode(runMode);
        }
}