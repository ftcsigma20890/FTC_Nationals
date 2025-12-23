package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class ElevatorWithMagneticSwitchAuto extends LinearOpMode {

    private DcMotor elevator;
    private DigitalChannel magSwitch;

    private final double HOLD_POWER = 0.15;
    private boolean holding = false;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        magSwitch = hardwareMap.get(DigitalChannel.class, "magSwitch");

        elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        magSwitch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()) {

            boolean magnetDetected = !magSwitch.getState();   // LOW = detected

            //==========================================
            // 1️⃣ MOVE UP UNTIL MAGNET DETECTED
            //==========================================
            holding = false;
            while (opModeIsActive() && !magnetDetected) {
                elevator.setPower(0.5);
                magnetDetected = !magSwitch.getState();
            }

            //==========================================
            // 2️⃣ MAGNET DETECTED → HOLD POSITION
            //==========================================
            elevator.setPower(HOLD_POWER);
            holding = true;
            sleep(300);     // hold for 0.3 sec

            //==========================================
            // 3️⃣ MOVE DOWN FOR 0.3 SEC
            //==========================================
            holding = false;
            elevator.setPower(-0.3);
            sleep(300);
            elevator.setPower(0);

            // After down, cycle repeats
        }
    }
}
