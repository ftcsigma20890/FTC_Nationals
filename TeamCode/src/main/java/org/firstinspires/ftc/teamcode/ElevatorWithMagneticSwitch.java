package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class ElevatorWithMagneticSwitch extends LinearOpMode {

    private DcMotor elevator;
    private DigitalChannel magSwitch;

    private final double HOLD_POWER = 0.15;   // small upward hold
    private boolean holding = false;          // to track holding mode

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
            // MOVE UP
            //==========================================
            if (gamepad1.dpad_up) {

                holding = false;   // disable hold when moving

                if (!magnetDetected) {
                    elevator.setPower(0.5);        // go up
                } else {
                    elevator.setPower(HOLD_POWER); // apply hold
                    holding = true;
                }
            }

            //==========================================
            // MOVE DOWN
            //==========================================
            else if (gamepad1.dpad_down) {

                holding = false;   // disable hold

                elevator.setPower(-0.5);  // go down
            }

            //==========================================
            // NO BUTTON → HOLD ONLY IF MAGNET WAS TOUCHED
            //==========================================
            else {

                if (holding) {
                    elevator.setPower(HOLD_POWER);  // keep elevator up
                } else {
                    elevator.setPower(0);           // idle normal
                }
            }

            telemetry.addData("Magnet", magnetDetected ? "DETECTED" : "NOT");
            telemetry.addData("Encoder", elevator.getCurrentPosition());
            telemetry.addData("Hold Active", holding);
            telemetry.update();
        }
    }
}
