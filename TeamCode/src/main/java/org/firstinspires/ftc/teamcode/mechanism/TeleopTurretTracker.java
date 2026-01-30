//package org.firstinspires.ftc.teamcode.mechanism;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//public class TeleopTurretTracker {
//
//    private DcMotorEx turret;
//    private Limelight3A limelight;
//    private Telemetry telemetry;
//
//    /* Small limits around TeleOp zero */
//    private static final int RIGHT_LIMIT = 400;
//    private static final int LEFT_LIMIT  = -400;
//
//    /* Control */
//    private static final double KP = 0.035;
//    private static final double MAX_POWER = 0.4;
//    private static final double DEADBAND = 1.2;
//
//    private boolean turretTrackingEnabled = false;
//    private int encoderOffset = 0;
//
//    public TeleopTurretTracker(HardwareMap hw, Telemetry telemetry) {
//        this.telemetry = telemetry;
//
//        turret = hw.get(DcMotorEx.class, "turret");
//        limelight = hw.get(Limelight3A.class, "limelight");
//
//        turret.setDirection(DcMotorSimple.Direction.REVERSE);
//        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        limelight.start();
//    }
//
//    /** Call ONCE at TeleOp start */
//    public void setCurrentAsZero() {
//        encoderOffset = turret.getCurrentPosition();
//    }
//
//    public void enableTeleopTracking() {
//        turretTrackingEnabled = true;
//    }
//
//    public void disableTeleopTracking() {
//        turretTrackingEnabled = false;
//        stop();
//    }
//
//    public void update() {
//        if (!turretTrackingEnabled) return;
//
//        LLResult result = limelight.getLatestResult();
//
//        if (result == null || !result.isValid()) {
//            stop();
//            telemetry.addLine("No AprilTag");
//            return;
//        }
//
//        double tx = result.getTx();
//
//        if (Math.abs(tx) < DEADBAND) {
//            stop();
//            telemetry.addLine("AprilTag Locked");
//            return;
//        }
//
//        double power = tx * KP;
//        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
//
//        int pos = turret.getCurrentPosition() - encoderOffset;
//
//        if ((pos <= LEFT_LIMIT && power < 0) || (pos >= RIGHT_LIMIT && power > 0)) {
//            power = 0;
//        }
//
//        turret.setPower(power);
//
//        telemetry.addData("TX", tx);
//        telemetry.addData("Turret Pos (rel)", pos);
//        telemetry.addData("Turret Pos (abs)", turret.getCurrentPosition());
//        telemetry.addData("Power", power);
//    }
//
//    public void stop() {
//        turret.setPower(0);
//    }
//
//    public int getRelativePosition() {
//        return turret.getCurrentPosition() - encoderOffset;
//    }
//
//    public int getAbsolutePosition() {
//        return turret.getCurrentPosition();
//    }
//}
