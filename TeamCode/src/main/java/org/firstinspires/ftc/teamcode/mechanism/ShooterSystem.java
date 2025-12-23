package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSystem {

    private DcMotorEx leftshoot, rightshoot;
    private DcMotor intake;
    private Servo lowerpush, middlepush, sorter;

    public ShooterSystem(HardwareMap hardwareMap) {
        // Hardware initialization
        leftshoot = hardwareMap.get(DcMotorEx.class, "leftshoot");
        rightshoot = hardwareMap.get(DcMotorEx.class, "rightshoot");
        intake = hardwareMap.get(DcMotor.class, "intake");
        lowerpush = hardwareMap.get(Servo.class, "lowerpush");
        middlepush = hardwareMap.get(Servo.class, "middlepush");
        sorter = hardwareMap.get(Servo.class, "sorter");

        // Directions
        rightshoot.setDirection(DcMotorSimple.Direction.REVERSE);
        leftshoot.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Encoder mode
        rightshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initial servo positions
        lowerpush.setPosition(0);
        middlepush.setPosition(0);
        sorter.setPosition(0.76);
    }

    // 🔹 Method to start intake
    public void setIntake(boolean on) {
        intake.setPower(on ? 1 : 0);
    }

    // 🔹 Main 3-ball shooting sequence
    public void threeShoot(LinearOpMode opMode) {
        double velocity = 1000;

        // Spin up shooters
        rightshoot.setVelocity(velocity);
        leftshoot.setVelocity(velocity);
        opMode.sleep(1200);

        // === SHOOT BALL 1 ===
        lowerpush.setPosition(0.4);
        opMode.sleep(800);
        lowerpush.setPosition(0.4);
        opMode.sleep(800);

        // === SHOOT BALL 2 ===
        lowerpush.setPosition(0.6);
        opMode.sleep(1500);
        lowerpush.setPosition(0.4);
        middlepush.setPosition(0.15);
        opMode.sleep(1000);

        // === SHOOT BALL 3 ===
        middlepush.setPosition(1);
        lowerpush.setPosition(0);
        opMode.sleep(1500);
        middlepush.setPosition(0);

        // Stop everything
        intake.setPower(0);
        rightshoot.setVelocity(0);
        leftshoot.setVelocity(0);
    }
}
