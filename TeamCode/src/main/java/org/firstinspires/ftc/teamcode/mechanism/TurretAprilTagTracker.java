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
//public class TurretAprilTagTracker {
//
//    /* ================= HARDWARE ================= */
//    private DcMotorEx turret;
//    private Limelight3A limelight;
//    private Telemetry telemetry;
//
//    /* ================= LIMITS ================= */
//    private static final int RIGHT_LIMIT = 1100;
//    private static final int LEFT_LIMIT  = -700;
//
//    /* ================= CONTROL ================= */
//    private static final double TRACK_POWER  = 0.55;
//    private static final double RETURN_POWER = 0.55;
//    private static final double DEADBAND = 1.2; // deg of tx
//
//    /* ================= STATE ================= */
//    private boolean trackingEnabled = false;
//    private boolean autoEnableTrackingAfterMove = false;
//
//    /* ================= CONSTRUCTOR ================= */
//    public TurretAprilTagTracker(HardwareMap hw, Telemetry telemetry) {
//        this.telemetry = telemetry;
//
//        turret = hw.get(DcMotorEx.class, "turret");
//        limelight = hw.get(Limelight3A.class, "limelight");
//
//        turret.setDirection(DcMotorSimple.Direction.REVERSE);
//        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // INIT zero = physical start
//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        limelight.start();
//    }
//
//    /* ================= TRACKING CONTROL ================= */
//    public void enableTracking() {
//        trackingEnabled = true;
//    }
//
//    public void disableTracking() {
//        trackingEnabled = false;
//        stop();
//    }
//
//    /* ================= AUTO: GO TO TICKS ================= */
//    public void goToTicks(int targetTicks) {
//        trackingEnabled = false;
//        autoEnableTrackingAfterMove = true;
//
//        targetTicks = Math.max(LEFT_LIMIT, Math.min(RIGHT_LIMIT, targetTicks));
//
//        turret.setTargetPosition(targetTicks);
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turret.setPower(TRACK_POWER);
//
//        telemetry.addData("Turret GoTo (ticks)", targetTicks);
//    }
//
//    /* ================= AUTO: RETURN TO ZERO ================= */
//    public void returnToZero() {
//        trackingEnabled = false;
//        autoEnableTrackingAfterMove = true;
//
//        turret.setTargetPosition(0);
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turret.setPower(RETURN_POWER);
//
//        telemetry.addLine("Turret Returning to Zero");
//    }
//
//    public void returnToRightLimit() {
//        trackingEnabled = false;
//        autoEnableTrackingAfterMove = true;
//
//        turret.setTargetPosition(RIGHT_LIMIT);
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turret.setPower(RETURN_POWER);
//
//        telemetry.addLine("Turret Returning to Right Limit");
//    }
//
//    public boolean isAtTarget() {
//        return !turret.isBusy();
//    }
//
//    /* ================= MAIN UPDATE ================= */
//    public void update() {
//
//        // Auto-enable tracking after movement
//        if (autoEnableTrackingAfterMove) {
//            if (!turret.isBusy()) {
//                autoEnableTrackingAfterMove = false;
//                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                trackingEnabled = true;
//                telemetry.addLine("Turret reached target → Tracking ENABLED");
//            }
//            return;
//        }
//
//        if (!trackingEnabled) return;
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
//        int currentPos = turret.getCurrentPosition();
//        int deltaTicks = (int) (tx * 8.0); // tune this
//        int targetPos = currentPos + deltaTicks;
//
//        targetPos = Math.max(LEFT_LIMIT, Math.min(RIGHT_LIMIT, targetPos));
//
//        turret.setTargetPosition(targetPos);
//        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        turret.setPower(TRACK_POWER);
//
//        telemetry.addData("TX", tx);
//        telemetry.addData("Turret Pos", currentPos);
//        telemetry.addData("Turret Target", targetPos);
//    }
//
//    /* ================= STOP ================= */
//    public void stop() {
//        turret.setPower(0);
//        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    /* ================= UTIL ================= */
//    public int getCurrentPosition() {
//        return turret.getCurrentPosition();
//    }
//}
