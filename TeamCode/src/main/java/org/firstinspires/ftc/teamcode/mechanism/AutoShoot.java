package org.firstinspires.ftc.teamcode.mechanism;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class AutoShoot {

    /* ================= HARDWARE ================= */
    public DcMotorEx leftshoot, rightshoot;
    public static double gear = 1.8;
    public DcMotor intake;
    public Servo holder, hood1, hood2, led;
    public Limelight3A limelight;
    public Telemetry telemetry;
    public static double holderShootPosition = 0.1;

    /* ================= STATE ================= */
    public enum State {IDLE, SPINUP, FEED, STOP}

    public State state = State.IDLE;
    public ElapsedTime timer = new ElapsedTime();

    /* ================= CONSTANTS ================= */
    public static double MIN_DISTANCE = 29.0;
    public static double MOUNT_ANGLE = 25;
    public static double LENS_HEIGHT = 15.5;
    public static double GOAL_HEIGHT = 29.0;
    public static double VELOCITY_TOLERANCE = 50;

    /* LED positions */
    public static double LED_BLUE = 0.63;

    /* ================= RAMP ================= */
    public static double RAMP_RATE = 1000; // ticks/sec²
    public double currentVelocity = 0;

    /* ================= TARGET ================= */
    public double targetVelocity = 0;
    public double hoodPos = 0;

    /* ================= CONSTRUCTOR ================= */
    public AutoShoot(HardwareMap hw, Telemetry telemetry) {

        this.telemetry = telemetry;

        leftshoot = hw.get(DcMotorEx.class, "leftshoot");
        rightshoot = hw.get(DcMotorEx.class, "rightshoot");
        intake = hw.get(DcMotor.class, "intake");

        holder = hw.get(Servo.class, "ramp");
        hood1 = hw.get(Servo.class, "Hood1");
        hood2 = hw.get(Servo.class, "Hood2");
        led = hw.get(Servo.class, "rgb");

        limelight = hw.get(Limelight3A.class, "limelight");

        leftshoot.setDirection(DcMotorSimple.Direction.FORWARD);
        rightshoot.setDirection(DcMotorSimple.Direction.REVERSE);

        leftshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftshoot.setVelocityPIDFCoefficients(450, 0, 12, 15.8);
        rightshoot.setVelocityPIDFCoefficients(450, 0, 12, 15.8);

        hood1.setDirection(Servo.Direction.REVERSE);
        hood2.setDirection(Servo.Direction.FORWARD);

        holder.setPosition(0);
        hood1.setPosition(0);
        hood2.setPosition(0);
        led.setPosition(0.0);
    }

    /* ================= DISTANCE ================= */
    public double getDistanceToTarget() {
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) {
            double angle = MOUNT_ANGLE - r.getTx();
            return (GOAL_HEIGHT - LENS_HEIGHT) / Math.tan(Math.toRadians(angle)) + 6;
        }
        return -1;
    }

    /* ================= VELOCITY MAP ================= */
    public double calculateVelocity(double d) {

        double velocity;

        // 🔒 HARD LOCK AFTER 130 inches
        if (d >= 140)
            velocity = 1250;
        else {
            d = Math.max(29, d);

            if (d <= 45)
                velocity = 850 + (900 - 850) * (d - 29) / (45 - 29);

            else if (d <= 70)
                velocity = 900 + (1000 - 900) * (d - 45) / (70 - 45);

            else
                // 65 -> 1050 , 130 -> 1230
                velocity = 1020 + (1250 - 990) * (d - 65) / (130 - 65);
        }

        // ✅ Round here
        return Math.round(velocity * gear);
    }

    /* ================= HOOD MAP ================= */
    public double calculateHood(double d) {
        d = Math.max(30, Math.min(140, d));

        if (d >= 65 && d <= 75) {
            return 0.7;
        }

        // 40 → 65  : 0.95 → 0.65
        if (d < 65) {
            return 0.95 + (0.65 - 0.95) * (d - 30) / (65 - 30);
        }

        // 75 → 130 : 0.65 → 0.08
        return 0.35 + (0.075 - 0.35) * (d - 75) / (130 - 75);
    }

    /* ================= VELOCITY CHECK ================= */
    public boolean atVelocity() {
        return Math.abs(leftshoot.getVelocity() - targetVelocity) < VELOCITY_TOLERANCE &&
                Math.abs(rightshoot.getVelocity() - targetVelocity) < VELOCITY_TOLERANCE;
    }

    /* ================= START PRE-RAMP ================= */
    public void startPreRamp(double preVelocity) {
        if (state == State.IDLE) {
            currentVelocity = Math.max(currentVelocity, preVelocity);
            leftshoot.setVelocity(currentVelocity);
            rightshoot.setVelocity(currentVelocity);
        }
    }

    public void stopPreRamp() {
        leftshoot.setPower(0);  // or whatever motor runs pre-ramp
        rightshoot.setPower(0);  // or whatever motor runs pre-ramp
    }

    /* ================= START SHOOT ================= */
    public void shootThreeBallsFromLimelight() {
        if (state != State.IDLE) return;

        double d = getDistanceToTarget();
        if (d < MIN_DISTANCE) return;

        targetVelocity = calculateVelocity(d); // already rounded
        hoodPos = calculateHood(d);

        // Ramp starts from actual velocity
        currentVelocity = getActualVelocity();

        hood1.setPosition(hoodPos);
        hood2.setPosition(hoodPos);
        holder.setPosition(holderShootPosition);

        timer.reset();
        state = State.SPINUP;
    }

    /* ================= UPDATE ================= */
    public void update() {
        LLResult r = limelight.getLatestResult();
        boolean hasTarget = r != null && r.isValid();

        // LED: only blue/off
        led.setPosition(hasTarget ? LED_BLUE : 0.0);
        double distance = getDistanceToTarget();

        switch (state) {
            case SPINUP:
                currentVelocity += RAMP_RATE * 0.02;
                currentVelocity = Math.min(currentVelocity, targetVelocity);

                // ✅ Apply rounded velocity to motors
                double roundedVelocity = Math.round(currentVelocity);
                leftshoot.setVelocity(roundedVelocity);
                rightshoot.setVelocity(roundedVelocity);

                if (timer.seconds() > 0.45 && atVelocity()) {
                    intake.setPower(1);
                    timer.reset();
                    state = State.FEED;
                }
                break;

            case FEED:
                if (timer.seconds() > 0.75) {
                    intake.setPower(0);
                    leftshoot.setVelocity(0);
                    rightshoot.setVelocity(0);
                    timer.reset();
                    state = State.STOP;
                }
                break;

            case STOP:
                if (timer.seconds() > 0.25) {
                    holder.setPosition(0);
                    state = State.IDLE;
                }
                break;
        }

        telemetry.addData("Has Target", hasTarget);
        telemetry.addData("Distance (in)", distance);
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Actual Vel", getActualVelocity());
    }

    /* ================= GETTERS ================= */
    public boolean isBusy() {
        return state != State.IDLE;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getActualVelocity() {
        return (leftshoot.getVelocity() + rightshoot.getVelocity()) / 2.0;
    }
}
