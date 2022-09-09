package org.firstinspires.ftc.teamcode.SwervedDifferential;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Joystick direction", group="SD")
public class JDirDrive extends OpMode {

    private DcMotor dreaptaSpate;
    private DcMotor dreaptaFata;
    private DcMotor stangaSpate;
    private DcMotor stangaFata;

    double gr1 = reg3Simpla(360, 1600, 1);


    boolean doneTurning = false;
    boolean sumSet = false;

    @Override
    public void init() {
        dreaptaSpate= hardwareMap.dcMotor.get("RightBackMotor");
        dreaptaFata= hardwareMap.dcMotor.get("RightFrontMotor");
        stangaFata = hardwareMap.dcMotor.get("LeftFrontMotor");
        stangaSpate = hardwareMap.dcMotor.get("LeftBackMotor");
    }

    @Override
    public void loop() {
        int leftOrientation = wheelOrientationReduced(stangaFata, stangaSpate);
        int rightOrientation = wheelOrientationReduced(dreaptaFata, dreaptaSpate);

        double jx = gamepad1.left_stick_x;
        double jy = -gamepad1.left_stick_y; // axele y-ului inversate
        double radians = Math.atan2(jx, jy);
        if(jx == 0 && jy*(-1) == 0) { // atan2(0,0) = pi => daca joystick-ul nu este atins are valorea de 180 (179,..)
            radians = 0;
            stangaFata.setPower(0);
            dreaptaFata.setPower(0);
        }
        double degrees = radians*57;
        if(degrees < 0) { // transformam din 0, 90, 180, -90 in 0, 90, 180, 270
            degrees += 360;
        }

        double turningLimit = gr1*degrees;

            if (Math.abs(leftOrientation - turningLimit) <= 30 || Math.abs(rightOrientation - turningLimit) <= 30) {
                stangaFata.setPower(0);
                dreaptaFata.setPower(0);
                if(jx != 0 && jy*(-1) != 0) {
                    doneTurning = true;
                }
            } else {
                stangaFata.setPower(jx / 3 + jy / 3); // putere la un singur motor pentru curbe
                dreaptaFata.setPower(jx / 3 + jy / 3);
                if(jx != 0 && jy*(-1) != 0) {
                    doneTurning = true;
                }
            }

            if(doneTurning) {
                stangaFata.setPower(jx/3);
                stangaSpate.setPower(jx/3);
                dreaptaFata.setPower(jx/3);
                dreaptaSpate.setPower(jx/3);
            }
// TODO invartim numai o roata
//            if(Math.abs(leftOrientation-rightOrientation) >= 5) {
//                int sum = 0;
//                sum = (leftOrientation + rightOrientation) / 2;
//                    if (sum < leftOrientation) {
//                        stangaFata.setPower(0.4);
//                        stangaSpate.setPower(0);
//                    } else if (sum > leftOrientation) {
//                        stangaFata.setPower(-0.4);
//                        stangaSpate.setPower(0);
//                    }
//            } else {
//                stangaSpate.setPower(0);
//                stangaFata.setPower(0);
//                dreaptaFata.setPower(0);
//                dreaptaSpate.setPower(0);
//            }
//            if(gamepad1.x) {
//                sumSet = false;
//            }

        if(Math.abs(leftOrientation-rightOrientation) >= 10) {
            if(leftOrientation < rightOrientation) {
                stangaFata.setPower(0.4);
                stangaSpate.setPower(-0.4);
                stangaFata.setPower(0);
                stangaSpate.setPower(0);
            }
            else if(rightOrientation < leftOrientation) {
                dreaptaFata.setPower(0.4);
                dreaptaSpate.setPower(-0.4);
                dreaptaSpate.setPower(0);
                dreaptaFata.setPower(0);
            }
        }


        // TODO functie pentru directia buna de intoarcere (mai apropiat de limita)

        telemetry.addData("left orientation", leftOrientation);
        telemetry.addData("right orientation", rightOrientation);
        telemetry.addData("degrees", degrees);
        telemetry.addData("radians", radians);
        telemetry.addData("turning limit", turningLimit);
        telemetry.addData("done turning", doneTurning);
        telemetry.update();
    }

    private int wheelOrientationReduced(DcMotor motor1, DcMotor motor2) {
        int poz1 = motor1.getCurrentPosition();
        int poz2 = motor2.getCurrentPosition();
        int dif = (poz1 - poz2)/2;
        int orientation = Math.abs(dif)%1600;

        return orientation;
    }

    private double reg3Simpla(double n1, double x1, double n2) {
        // n1...x1
        // n2...x2 (output)
        return (x1*n2)/n1;
    }

    void setEncoderMode(DcMotor.RunMode runMode) {
        dreaptaFata.setMode(runMode);
        dreaptaSpate.setMode(runMode);

        stangaFata.setMode(runMode);
        stangaSpate.setMode(runMode);
    }

}
