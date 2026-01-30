package org.firstinspires.ftc.teamcode.mechanism;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Configurable
public class LimelightAligner {

    private final DcMotor LF, LR, RF, RR;
    private final Limelight3A limelight;

    private static double kP = 0.02;
    private static double minPower = 0.15;
    private double threshold = 1.2; // degrees

    public LimelightAligner(HardwareMap hardwareMap) {

        LF = hardwareMap.get(DcMotor.class, "LF");
        LR = hardwareMap.get(DcMotor.class, "LR");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RR = hardwareMap.get(DcMotor.class, "RR");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LR.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

    }

    /* ================= ALIGN LOGIC ================= */
    public void alignToAprilTag() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double tx = result.getTy();
            double turnPower = kP * tx;

            if (Math.abs(tx) <= threshold) {
                stopMotors();
                return;
            }

            if (Math.abs(turnPower) < minPower) {
                turnPower = Math.signum(turnPower) * minPower;
            }

            LF.setPower(turnPower);
            LR.setPower(turnPower);
            RF.setPower(-turnPower);
            RR.setPower(-turnPower);

        } else {
            stopMotors();
        }
    }

    /* ================= ALIGN CHECK ================= */
    public boolean isAligned() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        return Math.abs(result.getTy()) <= threshold;
    }

    /* ================= STOP ================= */
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
