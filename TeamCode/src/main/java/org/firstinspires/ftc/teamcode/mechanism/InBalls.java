package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class InBalls {

    private DcMotor intake;
    private Servo holder;

    public InBalls(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        holder = hardwareMap.get(Servo.class, "holder");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        holder.setPosition(0); // door closed initially
    }

    // === Start intake (collect balls) ===
    public void startIntake(double power, LinearOpMode opMode) {
        holder.setPosition(0.35); // ensure door stays closed
        opMode.sleep(200);
        intake.setPower(power);
    }

    // === Stop intake (keep door closed) ===
    public void stopIntake() {
        intake.setPower(0);
        // door remains closed, no movement here
    }
}
