package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp (name = "Shooooot")
public class SyncShoot extends LinearOpMode {
    DcMotorEx leftshoot, rightshoot;
    DcMotor intake;
    Servo lowerpush, middlepush,sorter;


    @Override
    public void runOpMode() throws InterruptedException {
        leftshoot = hardwareMap.get(DcMotorEx.class,"leftshoot");
        rightshoot = hardwareMap.get(DcMotorEx.class,"rightshoot");
        intake = hardwareMap.get(DcMotor.class,"intake");
        lowerpush = hardwareMap.get(Servo.class,"lowerpush");
        middlepush = hardwareMap.get(Servo.class,"middlepush");
        sorter = hardwareMap.get(Servo.class,"sorter");
        rightshoot.setDirection(DcMotorSimple.Direction.REVERSE);
        leftshoot.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        rightshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightshoot.setVelocityPIDFCoefficients(20,1,3,11.7);
        leftshoot.setVelocityPIDFCoefficients(20,1,3,11.7);

        lowerpush.setPosition(0.98);
        middlepush.setPosition(0);
        sorter.setPosition(0.76);
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.left_bumper){
                ThreeShoot();
            }
            if(gamepad1.right_bumper){
                intake.setPower(0.7);
            }else {
                intake.setPower(0);
            }

        }

    }
    public void ThreeShoot(){
        double velocity = 1000;
        // Step 1: Spin up shooter motors
        rightshoot.setVelocity(velocity);
        leftshoot.setVelocity(velocity);
        sleep(1000); // Let the flywheels reach stable speed

        // === SHOOT BALL 1 ===
       // lowerpush.setPosition(0.4);
       // sleep(800);  // Allow push
        lowerpush.setPosition(0.42);  // Hold slightly forward (holding position)
        sleep(800);

        // === SHOOT BALL 2 ===
        //lowerpush.setPosition(0.6);  // Push further for 2nd ball
        //sleep(1500);
        //lowerpush.setPosition(0.4);  // Hold back again
        middlepush.setPosition(0.15);
        sleep(500);

        // === SHOOT BALL 3 (SYNC MIDDLE + LOWER) ===
        // Lower resets to 0, middle fires
        middlepush.setPosition(1);
        //lowerpush.setPosition(0);  // Middle pushes the 3rd ball
        sleep(800);
        middlepush.setPosition(0);  // Reset middle


        // Stop shooter and intake
        intake.setPower(0);
        rightshoot.setVelocity(0);
        leftshoot.setVelocity(0);

    }
}
