package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class AutoElevatorSequence extends LinearOpMode {

    private DcMotor elevator, intake;
    private DigitalChannel magSwitch;

    private final double HOLD_POWER = 0.15;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        intake = hardwareMap.get(DcMotor.class, "intake");
        magSwitch = hardwareMap.get(DigitalChannel.class, "magSwitch");

        elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        magSwitch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        // -------------------------------
        // 🔥 AUTOMATIC SEQUENCE
        // -------------------------------

        runElevatorCycle();   // 1
        intakePulse();        // intake on → off

        runElevatorCycle();   // 2
        intakePulse();        // intake on → off

        runElevatorCycle();   // 3
    }


    //====================================================
    // 🔥 ELEVATOR CYCLE METHOD
    //====================================================
    private void runElevatorCycle() {

        // Move UP until magnet detected
        while (opModeIsActive() && !isMagnetDetected()) {
            elevator.setPower(0.5);
        }

        // Magnet detected → HOLD
        elevator.setPower(HOLD_POWER);
        sleep(300);

        // Move DOWN for 0.3 sec
        elevator.setPower(-0.3);
        sleep(300);

        // Stop elevator
        elevator.setPower(0);
    }

    //====================================================
    // 🔥 MAGNET DETECTION
    //====================================================
    private boolean isMagnetDetected() {
        return !magSwitch.getState();
    }

    //====================================================
    // 🔥 INTAKE SHORT PULSE (ON → WAIT → OFF)
    //====================================================
    private void intakePulse() {
        intake.setPower(1.0);   // ON
        sleep(200);             // run for 0.2 sec
        intake.setPower(0);     // OFF
    }
}
