//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.mechanism.AutoShoot;
//import org.firstinspires.ftc.teamcode.mechanism.InBalls;
//
//@TeleOp(name = "ManualShootSimple")
//public class ManualShoot extends LinearOpMode {
//
//    DcMotorEx rightshoot, leftshoot;
//    DcMotor intake;
//    Servo holder;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Hardware mapping
//     intake = hardwareMap.get(DcMotor.class,"intake");
//     rightshoot = hardwareMap.get(DcMotorEx.class,"rightshoot");
//     leftshoot = hardwareMap.get(DcMotorEx.class,"leftshoot");
//     holder =hardwareMap.get(Servo.class,"holder");
//    // intake = hardwareMap.get(DcMotor.class,"intake");
//        rightshoot.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftshoot.setDirection(DcMotorSimple.Direction.FORWARD);
//     intake.setDirection(DcMotorSimple.Direction.FORWARD);
//        AutoShoot shooting = new AutoShoot(hardwareMap);
//        holder.setPosition(0);
//
//
//
//        waitForStart();
//
//
//            // Shooter on when left bumper held
//            while (opModeIsActive()) {
//              if(gamepad1.left_bumper){
//                  intake.setPower(0.7);
//
//              }else {
//                  intake.setPower(0);
//              }
//              if(gamepad1.right_bumper){
//                  rightshoot.setPower(0.45);
//                  leftshoot.setPower(0.45);
//              }else {
//                  rightshoot.setPower(0);
//                  leftshoot.setPower(0);
//
//              }
//              holder.setPosition(gamepad1.right_trigger);
//
//              }
//
//
//
//
//
//        }
//    }
//
