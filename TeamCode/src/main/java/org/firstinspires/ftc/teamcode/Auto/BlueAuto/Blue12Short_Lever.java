package org.firstinspires.ftc.teamcode.Auto.BlueAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanism.AutoShoot;
import org.firstinspires.ftc.teamcode.mechanism.TurretAprilTagTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Short -12 Lever")
public class Blue12Short_Lever extends LinearOpMode {

    /* ================= HARDWARE ================= */
    private DcMotor intake;
    private CRServo gripwheel;

    private Limelight3A limelight;
    private TurretAprilTagTracker turretTracker; // ✅ DC motor turret

    private AutoShoot shooter;
    private Follower follower;

    @Override
    public void runOpMode() {

        /* ================= INIT ================= */
        follower = Constants.createFollower(hardwareMap);
        shooter = new AutoShoot(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turretTracker = new TurretAprilTagTracker(hardwareMap, telemetry); // DC motor turret
        turretTracker.enableTracking(); // ✅ enable tracking

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        gripwheel = hardwareMap.get(CRServo.class, "gripwheel");

        Pose startPose = new Pose(34.000, 137.500, Math.toRadians(180));
        follower.setStartingPose(startPose);

        /* ================= PATHS ================= */
        PathChain Path1 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(34.000, 137.500), new Pose(53.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        PathChain Path2 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(53.000, 84.000), new Pose(15.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain Path3 = new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(15.000, 84.000),
                                new Pose(18.000, 80.000),
                                new Pose(14.000, 76.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        PathChain Path4 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(14.000, 76.000), new Pose(53.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        PathChain Path5 = new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(53.000, 84.000),
                                new Pose(44.000, 59.000),
                                new Pose(37.000, 59.000),
                                new Pose(9.000, 59.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain Path6 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(9.000, 59.000), new Pose(53.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain Path7 =new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(53.000, 84.000),
                                new Pose(57.000, 37.000),
                                new Pose(41.000, 35.000),
                                new Pose(8.972, 34.991)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        PathChain Path8 =new PathBuilder(follower)
                .addPath(
                        new BezierLine(new Pose(8.972, 34.991), new Pose(53.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();







        /* ================= START ================= */
        limelight.start();
        limelight.pipelineSwitch(2);
        waitForStart();
        turretTracker.goToTicks(280);

        /* ================= AUTON SEQUENCE ================= */
        shooter.startPreRamp(500);
        runPath(Path1, 1, 0);
        // Shoot preload
        shootAndWait();
        intake.setPower(0.8);
        gripwheel.setPower(-0.5);
        shooter.startPreRamp(500);
        runPath(Path2, 0.65, 0);
        //Collect balls
        intake.setPower(0);
        gripwheel.setPower(0);

        runPath(Path3, 1, 0);
        //Open classifier

        runPath(Path4, 1, 0);
        //Shoot
       shootAndWait();
        intake.setPower(0.8);
        gripwheel.setPower(-0.5);
        shooter.startPreRamp(500);
        runPath(Path5, 0.7, 0);
        intake.setPower(0);
        gripwheel.setPower(0);

        runPath(Path6, 1, 0);
        shootAndWait();
        intake.setPower(0.8);
        gripwheel.setPower(-0.5);
        shooter.startPreRamp(500);
        runPath(Path7,0.7,0);
        intake.setPower(0);
        gripwheel.setPower(0);

        runPath(Path8, 1, 0);
        shootAndWait();


        turretTracker.returnToZero();

        /* ================= END ================= */
        while (opModeIsActive()) {
            shooter.update();
            turretTracker.update(); // ✅ continuous tracking
            telemetry.addData("Turret Pos", turretTracker.getCurrentPosition());
            telemetry.update();
        }
    }

    /* ================= PATH RUNNER ================= */
    private void runPath(PathChain path, double power, long delayAfterMs) {
        follower.setMaxPower(power);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            turretTracker.update(); // ✅ DC motor turret tracks while moving
            shooter.update();
        }

        if (delayAfterMs > 0) sleep(delayAfterMs);
    }

    /* ================= SHOOT ================= */
    private void shootAndWait() {
        shooter.shootThreeBallsFromLimelight();

        while (opModeIsActive() && shooter.isBusy()) {
            shooter.update();
            turretTracker.update(); // ✅ DC motor turret keeps correcting
            telemetry.addData("Shooter", "ACTIVE");
            telemetry.addData("Target Vel", shooter.getTargetVelocity());
            telemetry.addData("Actual Vel", shooter.getActualVelocity());
            telemetry.update();
        }
    }
}
