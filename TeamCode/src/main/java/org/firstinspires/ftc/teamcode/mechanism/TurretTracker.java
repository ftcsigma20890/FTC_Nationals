package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretTracker {

    /* ================= HARDWARE ================= */
    private final Limelight3A limelight;
    private final CRServo turretLeft;
    private final CRServo turretRight;
    private final Telemetry telemetry;

    /* ================= TUNING ================= */
    private static final double KP = 0.02;
    private static final double MAX_POWER = 1.0;
    private static final double DEADBAND = 1.0; // degrees

    /* ================= STATE ================= */
    private boolean enabled = true;   // 🔥 RUNS FOREVER BY DEFAULT
    private boolean hasTarget = false;
    private double lastPower = 0;

    /* ================= CONSTRUCTOR ================= */
    public TurretTracker(HardwareMap hw, Telemetry telemetry) {

        this.telemetry = telemetry;

        limelight = hw.get(Limelight3A.class, "limelight");
        turretLeft  = hw.get(CRServo.class, "turretLeft");
        turretRight = hw.get(CRServo.class, "turretRight");

        limelight.pipelineSwitch(2);
        limelight.start();
    }

    /* ================= MAIN LOOP (CALL ALWAYS) ================= */
    public void update() {

        if (!enabled) {
            stop();
            return;
        }

        LLResult result = limelight.getLatestResult();
        double power = 0;

        if (result != null && result.isValid()) {

            hasTarget = true;
            double tx = result.getTx();

            if (Math.abs(tx) > DEADBAND) {
                power = KP * tx;
            }

            power = Range.clip(power, -MAX_POWER, MAX_POWER);
            power = -power; // invert if needed

        } else {
            hasTarget = false;
            power = 0;
        }

        lastPower = power;
        turretLeft.setPower(power);
        turretRight.setPower(power);

        telemetry.addData("Turret Enabled", enabled);
        telemetry.addData("Turret Target", hasTarget);
        telemetry.addData("Turret Power", "%.2f", power);
    }

    /* ================= CONTROL ================= */
    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        stop();
    }

    public void stop() {
        turretLeft.setPower(0);
        turretRight.setPower(0);
    }

    /* ================= INFO ================= */
    public boolean hasTarget() {
        return hasTarget;
    }

    public double getLastPower() {
        return lastPower;
    }
}
