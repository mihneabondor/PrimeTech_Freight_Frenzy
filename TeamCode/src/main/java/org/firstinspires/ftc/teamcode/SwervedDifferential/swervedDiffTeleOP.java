package org.firstinspires.ftc.teamcode.SwervedDifferential;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="SD Teleop", group="SD")

public class swervedDiffTeleOP extends OpMode
{

    private DcMotor dreaptaSpate;
    private DcMotor dreaptaFata;
    private DcMotor stangaSpate;
    private DcMotor stangaFata;

    private BNO055IMU imu;
    Orientation angles;

    int DEG_90_TARGET = 395;
    int DEG_0_TARGET = 0;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Before Initialization");

        dreaptaSpate= hardwareMap.dcMotor.get("RightBackMotor");
        dreaptaFata= hardwareMap.dcMotor.get("RightFrontMotor");
        stangaFata = hardwareMap.dcMotor.get("LeftFrontMotor");
        stangaSpate = hardwareMap.dcMotor.get("LeftBackMotor");

        setTargetPosition(DEG_0_TARGET);

        telemetry.addData("Status", "Initialized");

        setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    @Override
    public void loop()
    {
        double px =gamepad1.right_stick_x/2;
        double py;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        int pozDreaptaFata = dreaptaFata.getCurrentPosition();
        int pozDreaptaSpate = dreaptaSpate.getCurrentPosition();
        int difDreapta = (pozDreaptaFata - pozDreaptaSpate)/2;

        if(Math.abs(difDreapta)%1600 <= 1100 && Math.abs(difDreapta) >= 500) { // rotile se orienteaza automat pentru fata robotului
            py = -gamepad1.left_stick_y/2;
        } else {
            py = gamepad1.left_stick_y / 2;

        }

        dreaptaSpate.setPower(py-px);
        dreaptaFata.setPower(py+px);
        stangaSpate.setPower(py-px);
        stangaFata.setPower(py+px);

//        if(timer.seconds() >= 2) {// intoarce rotile dupa inactivity period
//            int ticksDiffRight = dif(dreaptaFata, dreaptaSpate);
//            int ticksDiffStanga = dif(stangaFata, stangaSpate);
//            if(motorsShouldReset(dreaptaFata, dreaptaSpate) == 1) {
//                setTargetPosition((ticksDiffRight - ticksDiffRight%1600));
//                setMotorPower(0.8);
//                setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
//            } else if(motorsShouldReset(dreaptaFata, dreaptaSpate) == 2) {
//                setTargetPosition((ticksDiffRight - ticksDiffRight%1600)+800);
//                setMotorPower(0.8);
//                setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//
//            if(motorsShouldReset(stangaFata, stangaSpate) == 1) {
//                setTargetPosition(ticksDiffStanga - ticksDiffStanga%1600);
//                setMotorPower(0.8);
//                setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
//            } else if(motorsShouldReset(stangaFata, stangaSpate) == 2) {
//                setTargetPosition((ticksDiffStanga - ticksDiffStanga % 1600)+800);
//                setMotorPower(0.8);
//                setEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//        }

        if(py != 0 || px != 0) {
            timer.reset();
        }

        if(gamepad1.a) {
            setMotorPower(0.8);
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

// TODO: PID Controller + tuning
        telemetry.addData("right front", dreaptaFata.getCurrentPosition());
        telemetry.addData("right back", dreaptaSpate.getCurrentPosition());
        telemetry.addData("left front", stangaFata.getCurrentPosition());
        telemetry.addData("left back", stangaSpate.getCurrentPosition());
        telemetry.addData("Wheel poz left", dif(stangaFata, stangaSpate));
        telemetry.addData("Wheel poz right", dif(dreaptaFata, dreaptaSpate));
        telemetry.update();
        }

        private void setTargetPosition(int target) {
            dreaptaSpate.setTargetPosition(target);
            dreaptaFata.setTargetPosition(-target);

            stangaSpate.setTargetPosition(target);
            stangaFata.setTargetPosition(-target);
        }

        private void setMotorPower(double power) {
            dreaptaFata.setPower(power);
            dreaptaSpate.setPower(power);
            stangaFata.setPower(power);
            stangaSpate.setPower(power);
        }


        private int dif(DcMotor motor1, DcMotor motor2) {
            int poz1 = motor1.getCurrentPosition();
            int poz2 = motor2.getCurrentPosition();
            int dif = (poz1 - poz2)/2;
            return Math.abs(dif)%1600;
        }

        private int motorsShouldReset(DcMotor motor1, DcMotor motor2) {
            int poz1 =  motor1.getCurrentPosition();
            int poz2 = motor2.getCurrentPosition();
            int dif = (poz1 - poz2)/2;
            if(Math.abs(dif) % 1600 <= 10) {
                return 0;
            } else if(Math.abs(dif)%1600 >= 400 && Math.abs(dif)%1600 <= 1200) {
                return 2;
            } else {
                return 1;
            }
        }

        void setEncoderMode(DcMotor.RunMode runMode) {
            dreaptaFata.setMode(runMode);
            dreaptaSpate.setMode(runMode);

            stangaFata.setMode(runMode);
            stangaSpate.setMode(runMode);
        }
}