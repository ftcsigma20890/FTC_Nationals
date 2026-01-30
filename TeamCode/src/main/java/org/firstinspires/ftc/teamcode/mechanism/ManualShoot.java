package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ManualShoot {

    /* ================= HARDWARE ================= */
    public DcMotorEx leftshoot, rightshoot;
    public DcMotor intake;
    public Servo holder, hood1, hood2, led;
    public static double holderShootPosition = 0.1;
    private Telemetry telemetry;

    /* ================= STATE ================= */
    public enum State {IDLE, SPINUP, FEED, STOP}
    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    /* ================= CONSTANTS ================= */
    private static final double VELOCITY_TOLERANCE = 60;
    private static final double RAMP_RATE = 800; // ticks/sec²

    /* LED positions */
    public static final double LED_BLUE = 0.63;

    /* ================= TARGET ================= */
    private double targetVelocity = 0;
    private double currentVelocity = 0;
    private double hoodPos = 0; // default, can be overridden

    /* ================= CONSTRUCTOR ================= */
    public ManualShoot(HardwareMap hw, Telemetry telemetry) {

        this.telemetry = telemetry;

        leftshoot = hw.get(DcMotorEx.class, "leftshoot");
        rightshoot = hw.get(DcMotorEx.class, "rightshoot");
        intake = hw.get(DcMotor.class, "intake");

        holder = hw.get(Servo.class, "ramp");
        hood1 = hw.get(Servo.class, "Hood1");
        hood2 = hw.get(Servo.class, "Hood2");
        led = hw.get(Servo.class, "rgb");

        leftshoot.setDirection(DcMotorSimple.Direction.FORWARD);
        rightshoot.setDirection(DcMotorSimple.Direction.REVERSE);

        leftshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightshoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftshoot.setVelocityPIDFCoefficients(500, 0, 0, 14);
        rightshoot.setVelocityPIDFCoefficients(500, 0, 0, 14);

        hood1.setDirection(Servo.Direction.REVERSE);
        hood2.setDirection(Servo.Direction.FORWARD);

        holder.setPosition(0);
        hood1.setPosition(hoodPos);
        hood2.setPosition(hoodPos);
        led.setPosition(0.0);
    }

    /* ================= VELOCITY CHECK ================= */
    private boolean atVelocity() {
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

    /* ================= START SHOOT (MANUAL VELOCITY) ================= */
    public void shootThreeBalls(double velocity, double hoodPosition) {
        if (state != State.IDLE) return;

        targetVelocity = velocity;
        hoodPos = hoodPosition;

        // Start ramp from current shooter speed
        currentVelocity = getActualVelocity();

        hood1.setPosition(hoodPos);
        hood2.setPosition(hoodPos);
        holder.setPosition(holderShootPosition);

        timer.reset();
        state = State.SPINUP;
    }

    /* ================= UPDATE ================= */
    public void update() {

        // LED ON when active shooting
        led.setPosition(state != State.IDLE ? LED_BLUE : 0.0);

        switch (state) {
            case SPINUP:
                currentVelocity += RAMP_RATE * 0.02;
                currentVelocity = Math.min(currentVelocity, targetVelocity);

                leftshoot.setVelocity(currentVelocity);
                rightshoot.setVelocity(currentVelocity);

                if (timer.seconds() > 0.75 && atVelocity()) {
                    intake.setPower(0.55);
                    timer.reset();
                    state = State.FEED;
                }
                break;

            case FEED:
                if (timer.seconds() > 2.0) {
                    intake.setPower(0);
                    leftshoot.setVelocity(0);
                    rightshoot.setVelocity(0);
                    timer.reset();
                    state = State.STOP;
                }
                break;

            case STOP:
                if (timer.seconds() > 0.4) {
                    holder.setPosition(0);
                    state = State.IDLE;
                }
                break;
        }

        telemetry.addData("ManualShoot State", state);
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Actual Vel", getActualVelocity());
    }

    /* ================= GETTERS ================= */
    public boolean isBusy() { return state != State.IDLE; }
    public double getTargetVelocity() { return targetVelocity; }
    public double getActualVelocity() { return (leftshoot.getVelocity() + rightshoot.getVelocity()) / 2.0; }
}
