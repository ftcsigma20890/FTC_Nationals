package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class ElevatorController {

    private DcMotor elevator, intake;
    private DigitalChannel magSwitch;

    private final double HOLD_POWER = 0.15;

    public ElevatorController(DcMotor elevator, DcMotor intake, DigitalChannel magSwitch) {
        this.elevator = elevator;
        this.intake = intake;
        this.magSwitch = magSwitch;
    }

    // ----------------------------------------------------
    // ⭐ Fully automatic elevator cycle (Thread.sleep ONLY)
    // ----------------------------------------------------
    public void runElevatorCycle() {

        // Move UP until magnet detected
        while (!isMagnetDetected()) {
            elevator.setPower(0.5);
        }

        // Magnet detected → HOLD position
        elevator.setPower(HOLD_POWER);
        safeSleep(300);

        // Move DOWN for 0.3 seconds
        elevator.setPower(-0.3);
        safeSleep(300);

        // Stop elevator
        elevator.setPower(0);
    }

    // ----------------------------------------------------
    // ⭐ Intake ON → short wait → OFF
    // ----------------------------------------------------
    public void intakePulse() {
        intake.setPower(1.0);
        safeSleep(200);
        intake.setPower(0);
    }

    // ----------------------------------------------------
    // ⭐ Magnet detection helper
    // ----------------------------------------------------
    public boolean isMagnetDetected() {
        return !magSwitch.getState();
    }

    // ----------------------------------------------------
    // ⭐ Safe sleep wrapper
    // ----------------------------------------------------
    private void safeSleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
