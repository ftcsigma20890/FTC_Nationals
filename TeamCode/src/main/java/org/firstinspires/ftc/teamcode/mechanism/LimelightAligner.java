package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightAligner {

    private final DcMotor LF, LR, RF, RR;
    private final Limelight3A limelight;

    private double kP = 0.02;
    private double minPower = 0.1;
    private double threshold = 1.0;

    public LimelightAligner(HardwareMap hardwareMap) {
        LF = hardwareMap.get(DcMotor.class, "LF");
        LR = hardwareMap.get(DcMotor.class, "LR");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RR = hardwareMap.get(DcMotor.class, "RR");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Motor directions (adjust if needed)
        LF.setDirection(DcMotor.Direction.REVERSE);
        LR.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.FORWARD);

        limelight.pipelineSwitch(6);
        limelight.start();
    }

    public void alignToAprilTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // horizontal offset
            double turnPower = kP * tx;

            // Deadband and minimum correction
            if (Math.abs(tx) <= threshold) {
                stopMotors();
                return;
            }

            if (Math.abs(turnPower) < minPower)
                turnPower = Math.signum(turnPower) * minPower;

            // Apply turning power
            LF.setPower(turnPower);
            LR.setPower(turnPower);
            RF.setPower(-turnPower);
            RR.setPower(-turnPower);
        } else {
            stopMotors();
        }
    }

    public void stopMotors() {
        LF.setPower(0);
        LR.setPower(0);
        RF.setPower(0);
        RR.setPower(0);
    }

    public void stopLimelight() {
        limelight.stop();
    }
}
