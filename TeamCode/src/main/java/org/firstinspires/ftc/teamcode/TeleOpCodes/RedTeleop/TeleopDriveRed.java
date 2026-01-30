//package org.firstinspires.ftc.teamcode.TeleOpCodes.RedTeleop;
//
//import com.pedropathing.follower.Follower;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.mechanism.AutoShoot;
//import org.firstinspires.ftc.teamcode.mechanism.LimelightAligner;
//import org.firstinspires.ftc.teamcode.mechanism.ManualShoot;
//import org.firstinspires.ftc.teamcode.mechanism.Parking;
//import org.firstinspires.ftc.teamcode.mechanism.TeleopTurretTracker;
//import org.firstinspires.ftc.teamcode.mechanism.TurretAprilTagTracker;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@TeleOp(name = "DriveRed", group = "Main")
//public class TeleopDriveRed extends LinearOpMode {
//
//    /* ================= HARDWARE ================= */
//    private Follower follower;
//    private DcMotor intake;
//    private CRServo gripwheel;
//
//    private AutoShoot shooter;
//    private Limelight3A limelight;
//    private LimelightAligner limelightAligner;
//    private TurretAprilTagTracker turretTracker;
//    private TeleopTurretTracker teleopTurretTracker;
//    private Parking parking;
//    private ManualShoot manualShoot;
//
//    /* ================= STATE ================= */
//    private double speed = 1;
//    private boolean aligning = false;
//    private boolean lastA = false;
//    private long alignStartTime = 0;
//
//    @Override
//    public void runOpMode() {
//
//        /* ================= INIT ================= */
//        intake = hardwareMap.get(DcMotor.class, "intake");
//        gripwheel = hardwareMap.get(CRServo.class, "gripwheel");
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//
//        follower = Constants.createFollower(hardwareMap);
//        shooter = new AutoShoot(hardwareMap, telemetry);
//        limelightAligner = new LimelightAligner(hardwareMap);
//        turretTracker = new TurretAprilTagTracker(hardwareMap, telemetry);
//        teleopTurretTracker = new TeleopTurretTracker(hardwareMap, telemetry);
//        parking = new Parking(hardwareMap);
//        manualShoot = new ManualShoot(hardwareMap, telemetry);
//
//        // Start with Auto tracker enabled
//        turretTracker.enableTracking();
//        limelight.pipelineSwitch(3);
//        limelight.start();
//
//        telemetry.addLine("TeleOp Ready");
//        telemetry.update();
//
//        follower.startTeleopDrive(true);
//        follower.update();
//
//        waitForStart();
//
//        // Set TeleOp relative zero from current turret position (after Auto)
//        teleopTurretTracker.setCurrentAsZero();
//
//        /* ================= LOOP ================= */
//        while (opModeIsActive()) {
//
//            // ===== UPDATE SUBSYSTEMS =====
//            shooter.update();
//            manualShoot.update();
//            turretTracker.update();         // Always update Auto tracker
//            teleopTurretTracker.update();   // Always update TeleOp tracker if enabled
//
//            // ===== LIMELIGHT TARGET =====
//            LLResult r = limelight.getLatestResult();
//            boolean hasTarget = r != null && r.isValid();
//
//            // ===== DRIVE =====
//            double y = -gamepad1.left_stick_y;
//            double x = -gamepad1.left_stick_x;
//            double rx = -gamepad1.right_stick_x;
//
//            if (!aligning) {
//                follower.setTeleOpDrive(y, x, rx, true);
//                follower.setMaxPower(speed);
//                follower.update();
//            }
//
//            // ===== ALIGN BUTTON (A) =====
//            if (gamepad1.a && !lastA && !aligning) {
//                aligning = true;
//                alignStartTime = System.currentTimeMillis();
//            }
//            lastA = gamepad1.a;
//
//            if (aligning) {
//                limelightAligner.alignToAprilTag();
//
//                if (limelightAligner.isAligned() || System.currentTimeMillis() - alignStartTime > 1200) {
//                    limelightAligner.stopMotors();
//                    aligning = false;
//                }
//            }
//
//            // ===== INTAKE / PRE-RAMP =====
//            if (!shooter.isBusy() && !manualShoot.isBusy()) {
//
//                /* ===== INTAKE CONTROL ===== */
//                if (gamepad2.left_trigger > 0.1) {
//                    intake.setPower(1);
//                    gripwheel.setPower(-0.4);
//                }
//                else if (gamepad2.right_trigger > 0.1) {
//                    intake.setPower(-1);
//                    gripwheel.setPower(0.4);
//                }
//                else {
//
//                    intake.setPower(0);
//                    gripwheel.setPower(0);
//                }
//
//                /* ===== PRE-RAMP CONTROL ===== */
//                if (gamepad2.a) {
//                    shooter.startPreRamp(500);   // Turn ON pre-ramp
//                }
//                else if (gamepad2.b) {
//                    shooter.stopPreRamp();       // Turn OFF pre-ramp
//                }
//            }
//
//
//            // ===== PARK =====
//            if (gamepad1.dpad_up && !shooter.isBusy() && !manualShoot.isBusy()) {
//                parking.deploy();
//            } else if (gamepad1.dpad_down) {
//                parking.retract();
//            }
//
//            // ===== SHOOT =====
//            if (gamepad2.right_bumper && hasTarget && !shooter.isBusy() && !manualShoot.isBusy()) {
//                shooter.shootThreeBallsFromLimelight();
//            } else if (gamepad2.x && !shooter.isBusy() && !manualShoot.isBusy()) {
//                manualShoot.shootThreeBalls(1000*0.85, 0.65);
//            } else if (gamepad2.y && !shooter.isBusy() && !manualShoot.isBusy()) {
//                manualShoot.shootThreeBalls(1250*0.85, 0.075);
//            }
//
//            // ===== LED =====
//            if (shooter.isBusy() || manualShoot.isBusy()) {
//                shooter.led.setPosition(0.55);
//            } else {
//                shooter.led.setPosition(hasTarget ? shooter.LED_BLUE : 0.0);
//            }
//
//            // ===== TURRET CONTROL =====
//            // TeleOp tracking (small limits)
//            if (gamepad2.dpad_up) {
//                teleopTurretTracker.enableTeleopTracking();
//
//            }
//            // Auto tracking (absolute) return to zero
//            else if (gamepad2.dpad_left) {
//                teleopTurretTracker.disableTeleopTracking();
//                turretTracker.returnToZero();
//
//            }
//            // Auto tracking (absolute) return to right limit
//            else if (gamepad2.dpad_right) {
//                teleopTurretTracker.disableTeleopTracking();
//                turretTracker.returnToRightLimit();
//
//            }
//
//            // ===== TELEMETRY =====
//            telemetry.addData("Has Target", hasTarget);
//            telemetry.addData("Shooter Busy", shooter.isBusy());
//            telemetry.addData("Manual Busy", manualShoot.isBusy());
//            telemetry.addData("Turret Absolute Pos", turretTracker.getCurrentPosition());
//            telemetry.addData("Turret Relative Pos", teleopTurretTracker.getRelativePosition());
//            telemetry.update();
//        }
//
//        turretTracker.stop();
//        teleopTurretTracker.stop();
//    }
//
//}
