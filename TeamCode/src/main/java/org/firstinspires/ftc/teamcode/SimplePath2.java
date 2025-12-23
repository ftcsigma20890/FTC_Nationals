package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanism.AutoShoot;
import org.firstinspires.ftc.teamcode.mechanism.TurretTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "SimplePath2")
public class SimplePath2 extends LinearOpMode {

    private DcMotor intake;
    private CRServo gripwheel;
    private AutoShoot shooter;
    private TurretTracker turret;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        shooter  = new AutoShoot(hardwareMap, telemetry);
        turret   = new TurretTracker(hardwareMap, telemetry);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        Pose startPose = new Pose(56.000, 9.000, Math.toRadians(90));
        follower.setStartingPose(startPose);

        /* ================= PATHS ================= */

        PathChain Path1 = new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(56.000, 9.000),
                        new Pose(47.103, 36.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        PathChain Path2 = new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(47.103, 36.000),
                        new Pose(14.000, 36.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain Path3 = new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(14.000, 36.000),
                        new Pose(59.888, 37.234),
                        new Pose(57.196, 84.785)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        PathChain Path4 = new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(57.196, 84.785),
                        new Pose(47.551, 60.500)))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .build();

        PathChain Path5 = new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(47.551, 60.500),
                        new Pose(14.000, 60.500)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain Path6 = new PathBuilder(follower)
                .addPath(new BezierCurve(
                        new Pose(14.000, 60.500),
                        new Pose(50.243, 54.729),
                        new Pose(59.439, 84.785)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        PathChain Path7 = new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(59.439, 84.785),
                        new Pose(44.187, 84.785)))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .build();

        PathChain Path8 = new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(44.187, 84.785),
                        new Pose(14.000, 84.336)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain Path9 = new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(14.000, 84.336),
                        new Pose(57.196, 84.785)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        /* ================= START ================= */

        waitForStart();

        shooter.shootThreeBallsFromLimelight();
        runPath(Path1, 1);

        intake.setPower(1);
        gripwheel.setPower(1);
        runPath(Path2, 0.8);
        intake.setPower(0);
        gripwheel.setPower(0);

        runPath(Path3, 1);

        shooter.shootThreeBallsFromLimelight();
        runPath(Path4, 1);

        intake.setPower(1);
        gripwheel.setPower(1);
        runPath(Path5, 0.8);
        intake.setPower(0);
        gripwheel.setPower(0);

        runPath(Path6, 1);

        shooter.shootThreeBallsFromLimelight();
        runPath(Path7, 0.8);

        intake.setPower(1);
        gripwheel.setPower(1);
        runPath(Path8, 0.8);
        intake.setPower(0);
        gripwheel.setPower(0);

        runPath(Path9, 1);

        shooter.shootThreeBallsFromLimelight();

        /* ===== KEEP SYSTEMS ALIVE AT END ===== */
        while (opModeIsActive()) {
            turret.update();
            shooter.update();
            telemetry.update();
        }
    }

    /* ================= RUN PATH ================= */
    private void runPath(PathChain path, double power) {

        follower.setMaxPower(power);
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {

            follower.update();   // drive
            turret.update();     // turret tracking
            shooter.update();    // shooter FSM

            telemetry.addData("Target Vel", shooter.getTargetVelocity());
            telemetry.addData("Actual Vel", shooter.getActualVelocity());
            telemetry.update();
        }

    }
}
