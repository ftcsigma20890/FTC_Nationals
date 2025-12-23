package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoShoot {

    /* ================= HARDWARE ================= */
    private DcMotorEx leftshoot, rightshoot;
    private DcMotor intake;
    private CRServo gripwheel;
    private Servo holder, hood1, hood2;
    private Servo led;   // goBILDA RGB LED (servo port)

    private Limelight3A limelight;
    private Telemetry telemetry;

    /* ================= STATE ================= */
    private enum State { IDLE, SPINUP, FEED, STOP, DONE }
    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    /* ================= CONSTANTS ================= */
    private static final double MIN_DISTANCE = 29.0;
    private static final double DISTANCE_OFFSET = 5.0;

    private static final double MOUNT_ANGLE = 21.0;
    private static final double LENS_HEIGHT = 12.0;
    private static final double GOAL_HEIGHT = 29.0;

    private static final double VELOCITY_TOLERANCE = 40;

    /* ================= LED POSITIONS ================= */
    private static final double LED_RED    = 0.3; // no target
    private static final double LED_YELLOW = 0.4; // target but too close
    private static final double LED_BLUE   = 0.630; // target seen
    private static final double LED_GREEN  = 0.550; // ready to shoot

    /* ================= VALUES ================= */
    private double targetVelocity = 0;
    private double hoodPos = 0;

    /* ================= CONSTRUCTOR ================= */
    public AutoShoot(HardwareMap hw, Telemetry telemetry) {

        this.telemetry = telemetry;

        leftshoot = hw.get(DcMotorEx.class, "leftshoot");
        rightshoot = hw.get(DcMotorEx.class, "rightshoot");
        intake = hw.get(DcMotor.class, "intake");
        gripwheel = hw.get(CRServo.class, "gripwheel");

        holder = hw.get(Servo.class, "holder");
        hood1 = hw.get(Servo.class, "Hood1");
        hood2 = hw.get(Servo.class, "Hood2");
        led = hw.get(Servo.class, "rgb");

        limelight = hw.get(Limelight3A.class, "limelight");

        leftshoot.setDirection(DcMotorSimple.Direction.FORWARD);
        rightshoot.setDirection(DcMotorSimple.Direction.REVERSE);

        leftshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftshoot.setVelocityPIDFCoefficients(220, 0, 22, 12.2);
        rightshoot.setVelocityPIDFCoefficients(220, 0, 22, 12.2);

        hood1.setDirection(Servo.Direction.REVERSE);
        hood2.setDirection(Servo.Direction.FORWARD);

        holder.setPosition(0);
        hood1.setPosition(0);
        hood2.setPosition(0);
        led.setPosition(LED_RED);

        limelight.pipelineSwitch(2);
        limelight.start();
    }

    /* ================= DISTANCE ================= */
    public double getDistanceToTarget() {
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) {
            double angle = MOUNT_ANGLE + r.getTy();
            double d = (GOAL_HEIGHT - LENS_HEIGHT)
                    / Math.tan(Math.toRadians(angle));
            return d + DISTANCE_OFFSET;
        }
        return -1;
    }

    /* ================= VELOCITY MAP ================= */
    private double calculateVelocity(double d) {

        if (d <= 29) return 910;

        if (d <= 60)
            return 910 + (1050 - 910) * (d - 29) / (60 - 29);

        return 1050 + (1260 - 1050) * (d - 60) / (95 - 60);
    }

    /* ================= HOOD MAP ================= */
    private double calculateHood(double d) {

        if (d < 40) d = 40;
        if (d > 95) d = 95;

        return 0.45 + (0.25 - 0.45) * (d - 60) / (95 - 60);
    }

    /* ================= VELOCITY CHECK ================= */
    private boolean atVelocity() {
        return Math.abs(leftshoot.getVelocity() - targetVelocity) < VELOCITY_TOLERANCE &&
                Math.abs(rightshoot.getVelocity() - targetVelocity) < VELOCITY_TOLERANCE;
    }

    /* ================= AUTO SHOOT ================= */
    public void shootThreeBallsFromLimelight() {

        if (state != State.IDLE) return;

        double d = getDistanceToTarget();
        if (d < MIN_DISTANCE) return;

        targetVelocity = calculateVelocity(d);
        hoodPos = calculateHood(d);

        hood1.setPosition(hoodPos);
        hood2.setPosition(hoodPos);
        holder.setPosition(0.25);

        leftshoot.setVelocity(targetVelocity);
        rightshoot.setVelocity(targetVelocity);

        timer.reset();
        state = State.SPINUP;
    }

    /* ================= UPDATE ================= */
    public void update() {

        LLResult r = limelight.getLatestResult();
        boolean hasTarget = r != null && r.isValid();
        double distance = hasTarget ? getDistanceToTarget() : -1;

        /* ---------- LED LOGIC (CORRECT ORDER) ---------- */
        if (!hasTarget) {
            led.setPosition(LED_RED);
        }
        else if (distance < MIN_DISTANCE) {
            led.setPosition(LED_YELLOW);
        }
        else if (atVelocity()) {
            led.setPosition(LED_GREEN);
        }
        else {
            led.setPosition(LED_BLUE);
        }

        /* ---------- STATE MACHINE ---------- */
        switch (state) {

            case SPINUP:
                if (timer.seconds() > 1.5 && atVelocity()) {
                    intake.setPower(0.65);
                    gripwheel.setPower(-1);
                    timer.reset();
                    state = State.FEED;
                }
                break;

            case FEED:
                if (timer.seconds() > 2.0) {
                    intake.setPower(0);
                    gripwheel.setPower(0);
                    leftshoot.setPower(0);
                    rightshoot.setPower(0);
                    timer.reset();
                    state = State.STOP;
                }
                break;

            case STOP:
                if (timer.seconds() > 0.8) {
                    holder.setPosition(0);
                    state = State.DONE;
                }
                break;

            case DONE:
                state = State.IDLE;
                break;
        }
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

    public double getLeftVelocity() {
        return leftshoot.getVelocity();
    }

    public double getRightVelocity() {
        return rightshoot.getVelocity();
    }
}
