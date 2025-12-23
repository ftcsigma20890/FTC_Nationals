package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretShoot {

    private DcMotor rightShoot, leftShoot, intake;
    private Servo linkage, backPush;

    public TurretShoot(HardwareMap hardwareMap) {
        rightShoot = hardwareMap.get(DcMotor.class, "rightshoot");
        leftShoot = hardwareMap.get(DcMotor.class, "leftshoot");
        intake = hardwareMap.get(DcMotor.class, "intake");
        backPush = hardwareMap.get(Servo.class, "backpush");
        linkage = hardwareMap.get(Servo.class, "linkage");

        rightShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShoot.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    // SHOOTS 3 BALLS ONE BY ONE
    public void shootThreeBalls(double power) {
        try {
            // Start flywheels
            rightShoot.setPower(power);
            leftShoot.setPower(power);
            Thread.sleep(1500); // spin-up

            // OPEN linkage ONCE at start
            linkage.setPosition(0.0);     // your open position

            // Shoot 3 balls individually
            shootOneBall();
            Thread.sleep(500);

            shootOneBall();
            Thread.sleep(500);
            intake.setPower(1);
            Thread.sleep(200);

            shootOneBall();
            Thread.sleep(500);
            intake.setPower(0);

            // Stop motors
            rightShoot.setPower(0);
            leftShoot.setPower(0);
            intake.setPower(0);

            // CLOSE linkage ONCE after all 3 balls
            linkage.setPosition(0.25);    // your close position

        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // Shoots ONE ball
    private void shootOneBall() throws InterruptedException {

        // Open backpush → allow only 1 ball
        backPush.setPosition(0.27);
        Thread.sleep(250);

        // Push ball using intake
        intake.setPower(0.7);
        Thread.sleep(250);

        // Stop feeding & block next ball
        intake.setPower(0);
        backPush.setPosition(0.2); // closed
        Thread.sleep(200);
    }
}
