// package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import java.util.Locale;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.util.Range;

// import com.qualcomm.hardware.bosch.BNO055IMU;
// import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
// import org.firstinspires.ftc.robotcore.external.Func;
// import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import org.firstinspires.ftc.robotcore.external.navigation.Position;
// import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// @TeleOp @Disabled

// public class TeleOPMecanum extends LinearOpMode
// {
    
//     private DcMotor stangaFata;
//     private DcMotor dreaptaFata;
//     private DcMotor stangaSpate;
//     private DcMotor dreaptaSpate;
    
//     A_hardwareMap robot = new A_hardwareMap();
    
//     public  double r = 0.5;
    
//     public double n=1;
    
//     @Override
//     public void init() {
//         telemetry.addData("Status", "Before Initialization");

//         robot.init(hardwareMap);

//         telemetry.addData("Status", "Initialized");
//     }
    
//     @Override
//     public void runOpMode()
//     {
//         stangaFata=hardwareMap.dcMotor.get("LeftFrontMotor");
//         dreaptaFata=hardwareMap.dcMotor.get("RightFrontMotor");
//         stangaSpate=hardwareMap.dcMotor.get("LeftBackMotor");
//         dreaptaSpate=hardwareMap.dcMotor.get("RightBackMotor");

//         dreaptaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         dreaptaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         stangaSpate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         stangaFata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//         dreaptaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         dreaptaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         stangaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         stangaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//         waitForStart();
//         while(opModeIsActive) {
//             double px = -gamepad1.left_stick_x;
//             double py = gamepad1.left_stick_y;
//             double pp = gamepad1.right_stick_x;

//             stangaSpate.setPower(py-px-pp);
//             stangaFata.setPower(py+px-pp);
//             dreaptaSpate.setPower(-py-px-pp);
//             dreaptaFata.setPower(-py+px-pp);
            
//             robot.carusel.setPower(gamepad2.right_trigger);
        
//         robot.ridicare.setPower(0.7 * gamepad2.left_stick_y);
        
        
//         if(gamepad2.dpad_left)
//             r=r+0.005;
//         if(gamepad2.dpad_right)
//             r=r-0.005;
//         robot.rotire.setPosition(r);
        
//         if(gamepad2.dpad_up)
//             robot.prindere.setPosition(0.26);
//         if(gamepad2.dpad_down)
//             robot.prindere.setPosition(0.5);
        
//         if(gamepad1.x)
//             n=1;
//         if(gamepad1.a)
//             n=0.50;
        
        
            
//         telemetry.addData("rotire:",r);
//         telemetry.addData("distanta:", robot.distanta.getDistance(DistanceUnit.CM));
//         telemetry.addData("n:", n);
//         telemetry.update();

//         }
//     }

// }

