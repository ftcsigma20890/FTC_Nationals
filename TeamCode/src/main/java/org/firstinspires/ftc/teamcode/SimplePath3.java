//package org.firstinspires.ftc.teamcode;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathBuilder;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.mechanism.AutoShoot;
//import org.firstinspires.ftc.teamcode.mechanism.InBalls;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//@Autonomous (name = "Simplepath3")
//public class SimplePath3 extends LinearOpMode {
//    DcMotor intake;
//
//    public void runOpMode() throws InterruptedException {
//        Follower follower = Constants.createFollower(hardwareMap);
//       AutoShoot shooter = new AutoShoot(hardwareMap);
//       intake = hardwareMap.get(DcMotor.class,"intake");
//       intake.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        //InBalls collection = new InBalls(hardwareMap);
//
//
//        Pose startPose = new Pose(56.000, 10.00, Math.toRadians(270));
//
//        follower.setStartingPose(startPose);
//        PathChain Path0 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(56.000, 10.000), new Pose(60.000, 15.000)))
//                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(295))
//                .build();
//
//        PathChain Path1 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(60.000, 15.000), new Pose(60.000, 43.000)))
//                .setLinearHeadingInterpolation(Math.toRadians(295), Math.toRadians(180))
//                .build();
//
//        PathChain Path2 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(60.000, 43.000), new Pose(15.000, 43.000)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        PathChain Path3 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(15.000, 43.000), new Pose(60.000, 15.000)))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(295))
//                .build();
//
//        PathChain Path4 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(60.000, 15.000), new Pose(60.000, 67.000)))
//                .setLinearHeadingInterpolation(Math.toRadians(295), Math.toRadians(180))
//                .build();
//
//
//        PathChain Path5 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(48.000, 67.000), new Pose(15.000, 67.000)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        PathChain Path6 = new PathBuilder(follower)
//                .addPath(
//                        new BezierCurve(
//                                new Pose(15.000, 67.000),
//                                new Pose(51.589, 49.121),
//                                new Pose(55.626, 83.000)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(310))
//                .build();
//
//        PathChain Path7 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(55.626, 83.000), new Pose(53, 93.000)))
//                .setLinearHeadingInterpolation(Math.toRadians(310), Math.toRadians(180))
//                .build();
//
//        PathChain Path8 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(53, 93.000), new Pose(17.000, 93.000)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//
//        PathChain Path9 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(17.000, 93.000), new Pose(55.626, 83.000)))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(310))
//                .build();
//
//        waitForStart();
//        runPath(follower,Path0,0.5,500);
//        shooter.shootThreeBalls(2000);
//        runPath(follower,Path1,1,0);
//        intake.setPower(1);
//        runPath(follower,Path2,0.5,500);
//        intake.setPower(0);
//        runPath(follower,Path3,1,0);
//        shooter.shootThreeBalls(2000);
//        runPath(follower,Path4,1,0);
//        intake.setPower(1);
//        runPath(follower,Path5,0.5,500);
//        intake.setPower(0);
//        runPath(follower,Path6,1,0);
//        shooter.shootThreeBalls(1500);
//        runPath(follower,Path7,0.5,0);
//        intake.setPower(1);
//        runPath(follower,Path8,0.5,500);
//        intake.setPower(0);
//        runPath(follower,Path9,1,0);
//        shooter.shootThreeBalls(1500);
//
//
//
//
//
//    }
//
//    private void runPath(Follower follower, PathChain path, double power, long delayAfterMs) {
//        follower.setMaxPower(power);
//        follower.followPath(path);
//        while (opModeIsActive() && follower.isBusy()) {
//            follower.update();
//        }
//        if (delayAfterMs > 0) sleep(delayAfterMs);
//    }
//}
