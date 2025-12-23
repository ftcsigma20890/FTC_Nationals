package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "ppg")
public class PPG_Shoot extends LinearOpMode {
    private DcMotorEx leftshoot, rightshoot;
    private DcMotor intake;
    private Servo lowerpush, middlepush, sorter;
    public void runOpMode() throws InterruptedException {
        leftshoot = hardwareMap.get(DcMotorEx.class,"leftshoot");
        rightshoot = hardwareMap.get(DcMotorEx.class,"rightshoot");
        intake = hardwareMap.get(DcMotor.class,"intake");
        lowerpush = hardwareMap.get(Servo.class,"lowerpush");
        middlepush = hardwareMap.get(Servo.class,"middlepush");
        sorter = hardwareMap.get(Servo.class,"sorter");
        rightshoot.setDirection(DcMotorSimple.Direction.REVERSE);
        leftshoot.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        rightshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftshoot.setVelocityPIDFCoefficients(20,1,20,5);
        rightshoot.setVelocityPIDFCoefficients(20,1,20,5);

        lowerpush.setPosition(0);
        middlepush.setPosition(0);


        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.left_bumper){
                PPG_Sort();
            }
            if(gamepad1.right_bumper){
                intake.setPower(1);
            }else {
                intake.setPower(0);
            }

        }

    }
    public void PPG_Sort(){
        double velocity = 1000;
        // Step 1: Spin up shooter motors
        rightshoot.setVelocity(velocity);
        leftshoot.setVelocity(velocity);
        sleep(3000); // Let the flywheels reach stable speed

        // === SHOOT BALL 1 ===
        lowerpush.setPosition(0.8);
        sleep(850);  // Allow push
        middlepush.setPosition(0.09);  // Hold slightly forward (holding position)
        sleep(800);

        // === SHOOT BALL 2 ===
         // Push further for 2nd ball
        lowerpush.setPosition(0);
        sleep(1000);
        middlepush.setPosition(1);
        sleep(800);
        middlepush.setPosition(0.09);
        sleep(800);

        // === SHOOT BALL 3 (SYNC MIDDLE + LOWER) ===
        // Lower resets to 0, middle fires
        sorter.setPosition(0.76);
        middlepush.setPosition(1);  // Middle pushes the 3rd ball
        sleep(1000);
        middlepush.setPosition(0);  // Reset middle
        sleep(1000);


        // Stop shooter and intake
        rightshoot.setVelocity(0);
        leftshoot.setVelocity(0);
    }

}
