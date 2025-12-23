package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import com.qualcomm.robotcore.util.Telemetry;

public class SmartShooter {

    /* ===================== HARDWARE ===================== */
    private DcMotorEx leftShoot, rightShoot;
    private DcMotor intake;
    private CRServo gripWheel;
    private Servo holder, hood1, hood2;
    private Telemetry telemetry; // Telemetry reference

    /* ===================== LIMELIGHT CONSTANTS ===================== */
    private static final double LIMELIGHT_MOUNT_ANGLE = 22.0; // deg
    private static final double LIMELIGHT_HEIGHT = 12.0;      // inches
    private static final double TARGET_HEIGHT = 29.5;         // inches

    public SmartShooter(HardwareMap hardwareMap) {
    }

    /* ===================== FSM ===================== */
    private enum State { IDLE, SPINUP, FEED, STOP, DONE }
    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    /* ===================== CONSTRUCTOR ===================== */
    public SmartShooter(com.qualcomm.robotcore.hardware.HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;

        leftShoot = hw.get(DcMotorEx.class, "leftshoot");
        rightShoot = hw.get(DcMotorEx.class, "rightshoot");
        intake = hw.get(DcMotor.class, "intake");
        gripWheel = hw.get(CRServo.class, "gripwheel");
        holder = hw.get(Servo.class, "holder");
        hood1 = hw.get(Servo.class, "Hood1");
        hood2 = hw.get(Servo.class, "Hood2");

        leftShoot.setDirection(DcMotorEx.Direction.FORWARD);
        rightShoot.setDirection(DcMotorEx.Direction.REVERSE);

        leftShoot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShoot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShoot.setVelocityPIDFCoefficients(200, 1, 20, 11.7);
        leftShoot.setVelocityPIDFCoefficients(200, 1, 20, 11.7);

        holder.setPosition(0);
        hood1.setPosition(0);
        hood2.setPosition(0);
    }

    /* ===================== PUBLIC METHODS ===================== */

    // Call this ONCE to start auto shooting using Limelight ty
    public void shootAuto(double ty) {
        if (state != State.IDLE) return;

        double distance = calculateDistance(ty);
        double velocity = velocityFromDistance(distance);
        double hoodPos = hoodFromDistance(distance);

        // Telemetry for debugging
        telemetry.addData("Limelight ty", ty);
        telemetry.addData("Calculated distance (inches)", distance);
        telemetry.addData("Shooter velocity (ticks/sec)", velocity);
        telemetry.addData("Hood position", hoodPos);
        telemetry.update();

        startShooting(velocity, hoodPos);
    }

    // Call this every loop to update the FSM
    public void update() {
        switch (state) {
            case SPINUP:
                if (timer.seconds() >= 2.0) {
                    intake.setPower(0.65);
                    gripWheel.setPower(-1);
                    timer.reset();
                    state = State.FEED;
                }
                break;
            case FEED:
                if (timer.seconds() >= 2.0) {
                    intake.setPower(0);
                    gripWheel.setPower(0);
                    leftShoot.setPower(0);
                    rightShoot.setPower(0);
                    timer.reset();
                    state = State.STOP;
                }
                break;
            case STOP:
                if (timer.seconds() >= 1.0) {
                    holder.setPosition(0);
                    state = State.DONE;
                }
                break;
            case DONE:
                state = State.IDLE;
                break;
            default:
                break;
        }
    }

    public boolean isBusy() {
        return state != State.IDLE;
    }

    /* ===================== INTERNAL METHODS ===================== */

    private void startShooting(double velocity, double hoodPos) {
        holder.setPosition(0.25);
        hood1.setPosition(hoodPos);
        hood2.setPosition(hoodPos);

        leftShoot.setVelocity(velocity);
        rightShoot.setVelocity(velocity);

        timer.reset();
        state = State.SPINUP;
    }

    // ------------------- Distance Calculation -------------------
    private double calculateDistance(double ty) {
        double angle = LIMELIGHT_MOUNT_ANGLE + ty;
        double distance = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(angle));
        return distance; // distance in inches
    }

    // ------------------- Velocity Formula -------------------
    private double velocityFromDistance(double d) {
        if (d <= 60) return 1050;             // plateau for close range
        return 1050 + 2.6667 * (d - 60);      // linear increase beyond 60 inches
    }

    // ------------------- Hood Position Formula -------------------
    private double hoodFromDistance(double d) {
        d = clamp(d, 9, 77);
        double hood = 0.25 + 0.00294 * (d - 9);
        return clamp(hood, 0.25, 0.45);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(v, max));
    }
}
