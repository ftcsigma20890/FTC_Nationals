package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp (name = "testShooter")

public class NewBot extends LinearOpMode {
    DcMotorEx rightShoot,leftShoot;
    DcMotor intake;
    Servo linkage,backpush;
    @Override
    public void runOpMode() throws InterruptedException {
        rightShoot = hardwareMap.get(DcMotorEx.class,"rightshoot");
        leftShoot = hardwareMap.get(DcMotorEx.class,"leftshoot");
        intake = hardwareMap.get(DcMotor.class,"intake");
        backpush = hardwareMap.get(Servo.class,"backpush");
        linkage = hardwareMap.get(Servo.class,"linkage");
        //comment
        rightShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShoot.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        linkage.setPosition(0.25);
        backpush.setPosition(0.2);


        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.right_bumper){
                rightShoot.setPower(1);
                leftShoot.setPower(1);
            }
            else {
                rightShoot.setPower(0);
                leftShoot.setPower(0);
            }
            if(gamepad1.dpad_up){
                backpush.setPosition(0.27);
                sleep(300);
                linkage.setPosition(0);
            }
            if (gamepad1.dpad_down){
                linkage.setPosition(0.25);
                sleep(300);
                backpush.setPosition(0.2);

            }
            if(gamepad1.left_bumper){
                intake.setPower(1);
            }else {
                intake.setPower(0);
            }
        }


    }
}
