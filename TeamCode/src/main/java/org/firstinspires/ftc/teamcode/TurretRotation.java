package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "Turret Rotation", group = "Test")
public class TurretRotation extends LinearOpMode {

    private Limelight3A limelight;

    // Turret CR servos
    private CRServo turretLeft;
    private CRServo turretRight;

    // -------- TUNING VALUES --------
    private static final double KP = 0.02;
    private static final double MAX_POWER = 1;
    private static final double DEADBAND = 1.0; // degrees

    @Override
    public void runOpMode() {

        // -------- Hardware --------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turretLeft  = hardwareMap.get(CRServo.class, "turretLeft");
        turretRight = hardwareMap.get(CRServo.class, "turretRight");

        telemetry.setMsTransmissionInterval(11);

        // AprilTag pipeline
        limelight.pipelineSwitch(2);
        limelight.start();

        telemetry.addLine("TurretRotation Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            double power = 0;

            if (result != null && result.isValid()) {

                double tx = result.getTx(); // horizontal offset

                if (Math.abs(tx) > DEADBAND) {
                    power = KP * tx;
                } else {
                    power = 0;
                }

                power = Range.clip(power, -MAX_POWER, MAX_POWER);
                power =-power;

                telemetry.addLine("Target Detected");
                telemetry.addData("tx (deg)", tx);
                telemetry.addData("Turret Power", power);

            } else {
                // No target → stop turret
                power = 0;
                telemetry.addLine("No Target Detected");
            }

            // Both servos rotate in SAME direction
            turretLeft.setPower(power);
            turretRight.setPower(power);

            telemetry.update();
        }
    }
}
