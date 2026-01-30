package org.firstinspires.ftc.teamcode.Auto.BlueAuto;

import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.mechanism.LimelightAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = " Blue Short - 12")
public class Blue12Short extends LinearOpMode {

    /* ================= HARDWARE ================= */
    private DcMotor intake;
    private LimelightAligner limelightAligner;
    private Limelight3A limelight;

    private AutoShoot shooter;
    private Follower follower;

    @Override
    public void runOpMode() {

        /* ================= INIT ================= */
        follower = Constants.createFollower(hardwareMap);
        shooter = new AutoShoot(hardwareMap, telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        Pose startPose = new Pose(33, 137.500, Math.toRadians(180));
        follower.setStartingPose(startPose);

        /* ================= PATHS ================= */
        PathChain Path1 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(33.000, 137.500),

                                new Pose(53.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        PathChain Path2 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(53.000, 84.000),

                                new Pose(15.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        PathChain Path3 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(14.000, 84.000),

                                new Pose(53.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
        PathChain Path4 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(53.000, 84.000),

                                new Pose(48.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        PathChain Path5 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(48.000, 60.000),

                                new Pose(10.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();


        PathChain Path6 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(10.000, 60.000),

                                new Pose(50.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        PathChain Path7 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(50.000, 84.000),

                                new Pose(48.000, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        PathChain Path8 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(48.000, 36.000),

                                new Pose(10.000, 36.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
        PathChain Path9 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(11.000, 36.000),

                                new Pose(50.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();







        /* ================= START ================= */
        limelight.start();
        limelight.pipelineSwitch(0);
        waitForStart();

        /* ================= AUTON SEQUENCE ================= */
        shooter.startPreRamp(500);
        runPath(Path1, 1, 0);
        shootAndWait();
        intake.setPower(1);

        shooter.startPreRamp(500);
        runPath(Path2, 0.8, 0);
        intake.setPower(0);


        runPath(Path3, 1, 0);
        shootAndWait();

        runPath(Path4, 1, 0);
        intake.setPower(1);

        shooter.startPreRamp(500);
        runPath(Path5, 0.8, 0);
        intake.setPower(0);


        runPath(Path6, 1, 0);
        shootAndWait();
        runPath(Path7, 1, 0);
        intake.setPower(1);
        shooter.startPreRamp(500);
        runPath(Path8, 0.8, 0);
        intake.setPower(0);
        runPath(Path9, 1, 0);
        shootAndWait();



        /* ================= END ================= */
        while (opModeIsActive()) {
            shooter.update();
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
        }

        if (delayAfterMs > 0) sleep(delayAfterMs);
    }

    /* ================= SHOOT ================= */
    private void shootAndWait() {
        shooter.shootThreeBallsFromLimelight();
        while (opModeIsActive() && shooter.isBusy()) {
            shooter.update();
            telemetry.addData("Shooter", "ACTIVE");
            telemetry.addData("Target Vel", shooter.getTargetVelocity());
            telemetry.addData("Actual Vel", shooter.getActualVelocity());
            telemetry.update();
        }
    }

}