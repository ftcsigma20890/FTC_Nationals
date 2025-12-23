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
//@Autonomous (name = "Simplepath4")
//public class SimplePath4 extends LinearOpMode {
//    DcMotor intake;
//
//    public void runOpMode() throws InterruptedException {
//        Follower follower = Constants.createFollower(hardwareMap);
//        AutoShoot shooter = new AutoShoot(hardwareMap);
//        intake = hardwareMap.get(DcMotor.class,"intake");
//        intake.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        //InBalls collection = new InBalls(hardwareMap);
//
//
//        Pose startPose = new Pose(35.000, 135.000, Math.toRadians(360));
//
//        follower.setStartingPose(startPose);
//        PathChain Path0 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(35.000, 135.000), new Pose(58.318, 96.000)))
//                .setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(320))
//                .build();
//
//        PathChain Path1 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(58.318, 96.000), new Pose(51.140, 96.393)))
//                .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(180))
//                .build();
//
//        PathChain Path2 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(51.140, 96.393), new Pose(12.000, 96.000)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        PathChain Path3 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(12.000, 96.000), new Pose(58.542, 102.000)))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(320))
//                .build();
//
//
//        PathChain Path4 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(58.542, 102.000), new Pose(59.000, 72.000)))
//                .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(180))
//                .build();
//
//
//        PathChain Path5 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(59.000, 72.000), new Pose(14.000, 72.)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        PathChain Path6 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(14.000, 72), new Pose(58.318, 102)))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(320))
//                .build();
//
//        PathChain Path7 = new PathBuilder(follower)
//
//                .addPath(new BezierLine(new Pose(58.318, 102), new Pose(59.664, 47)))
//                .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(180))
//                .build();
//
//        PathChain Path8 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(59.664, 47), new Pose(14.000, 47)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//
//        PathChain Path9 = new PathBuilder(follower)
//                .addPath(new BezierLine(new Pose(14.000, 47), new Pose(58.318, 102)))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(320))
//                .build();
//
//        waitForStart();
//        runPath(follower,Path0,1,500);
//        shooter.shootThreeBalls(0.45);
//        runPath(follower,Path1,0.5,1000);
//        intake.setPower(1);
//        runPath(follower,Path2,0.5,500);
//        intake.setPower(0);
//        runPath(follower,Path3,1,0);
//        shooter.shootThreeBalls(0.45);
//        runPath(follower,Path4,1,1000);
//        intake.setPower(1);
//        runPath(follower,Path5,0.5,500);
//        intake.setPower(0);
//        runPath(follower,Path6,1,0);
//        shooter.shootThreeBalls(0.3);
//        runPath(follower,Path7,1,1000);
//        intake.setPower(1);
//        runPath(follower,Path8,0.5,500);
//        intake.setPower(0);
//        runPath(follower,Path9,1,0);
//        shooter.shootThreeBalls(0.3);
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
