package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class TestMagSensor extends LinearOpMode {

    private DigitalChannel magSwitch;

    @Override
    public void runOpMode() throws InterruptedException {

        magSwitch = hardwareMap.get(DigitalChannel.class, "magSwitch");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()) {

            // Read sensor value
            boolean rawValue = magSwitch.getState();

            // If your switch gives LOW when magnet is near:
            boolean magnetDetected = !rawValue;

            telemetry.addData("Raw Value", rawValue);
            telemetry.addData("Magnet Detected?", magnetDetected);
            telemetry.update();
        }
    }
}
