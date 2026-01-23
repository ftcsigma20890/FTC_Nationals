package org.firstinspires.ftc.teamcode.Auto.RedAuto;

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

@Autonomous(name = "Red Alliance Short")
public class RedAllianceAutoShort extends LinearOpMode {

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

        Pose startPose = new Pose(110.953, 137.421, Math.toRadians(0));
        follower.setStartingPose(startPose);

        /* ================= PATHS ================= */
        PathChain Path1 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(110.953, 137.421),

                                new Pose(88.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();


        PathChain Path2 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(88.000, 84.000),

                                new Pose(128.869, 83.850)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        PathChain Path3 = new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(128.869, 83.850),
                                new Pose(124.551, 77.346),
                                new Pose(130.514, 74.131)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                .build();




        PathChain Path4 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(130.514, 74.131),

                                new Pose(88.187, 83.757)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                .build();


        PathChain Path5 = new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(88.187, 83.757),
                                new Pose(85.766, 55.178),
                                new Pose(104.589, 59.804),
                                new Pose(134.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();



        PathChain Path6 = new PathBuilder(follower)
                .addPath(
                        new BezierCurve(
                                new Pose(134.000, 60.000),
                                new Pose(106.266, 66.570),
                                new Pose(87.710, 83.869)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();


        PathChain Path7 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(87.710, 83.869),

                                new Pose(125.037, 68.523)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))

                .build();



        /* ================= START ================= */
        limelight.start();
        limelight.pipelineSwitch(3);
        waitForStart();
        turretTracker.goToTicks(-300);
        sleep(500);

        /* ================= AUTON SEQUENCE ================= */
        shooter.startPreRamp(500);
        runPath(Path1, 1, 5000);
        shootAndWait();
        intake.setPower(1);
        shooter.startPreRamp(500);
        runPath(Path2, 1, 2000);
        intake.setPower(0);
        runPath(Path3, 1, 500);
//
        runPath(Path4, 0.7, 1000);
        shootAndWait();
        intake.setPower(1);
        shooter.startPreRamp(500);
        runPath(Path5, 1, 0);
        intake.setPower(0);


        runPath(Path6,1,0);
        shootAndWait();


        runPath(Path7, 1, 0);


        turretTracker.returnToZero();

        /* ================= END ================= */
        while (opModeIsActive()) {
            shooter.update();
            turretTracker.update();
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
            shooter.update();
            turretTracker.update();
        }

        if (delayAfterMs > 0) sleep(delayAfterMs);
    }

    /* ================= SHOOT ================= */
    private void shootAndWait() {

        shooter.shootThreeBallsFromLimelight();

        // 🔥 FORCE INITIAL UPDATE CYCLES
        for (int i = 0; i < 5 && opModeIsActive(); i++) {
            shooter.update();
            turretTracker.update();
            sleep(20);
        }

        while (opModeIsActive() && shooter.isBusy()) {
            shooter.update();
            turretTracker.update();
            telemetry.addData("Shooter", "ACTIVE");
            telemetry.addData("Target Vel", shooter.getTargetVelocity());
            telemetry.addData("Actual Vel", shooter.getActualVelocity());
            telemetry.update();
        }
    }
}