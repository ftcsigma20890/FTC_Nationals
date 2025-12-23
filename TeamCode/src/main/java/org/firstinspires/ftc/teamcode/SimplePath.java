package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "SimplePath")
public class SimplePath extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);


        Pose startPose = new Pose(55.000, 15.00, Math.toRadians(295));

        follower.setStartingPose(startPose);


         PathChain Path2 = new PathBuilder(follower)
                .addPath(new BezierLine(new Pose(55.000, 15.000), new Pose(50.692, 35.215)))
                .setLinearHeadingInterpolation(Math.toRadians(295), Math.toRadians(180))
                .build();

        PathChain Path3 = new PathBuilder(follower)
                .addPath(new BezierLine(new Pose(50.692, 35.215), new Pose(18.393, 35.664)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

         PathChain Path4 = new PathBuilder(follower)
                .addPath(new BezierLine(new Pose(18.393, 35.664), new Pose(55.000, 15.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(295))
                .build();

         PathChain Path5 = new PathBuilder(follower)
                .addPath(new BezierLine(new Pose(55.000, 15.000), new Pose(45.757, 60.336)))
                .setLinearHeadingInterpolation(Math.toRadians(295), Math.toRadians(180))
                .build();

         PathChain Path6 = new PathBuilder(follower)
                .addPath(new BezierLine(new Pose(45.757, 60.336), new Pose(18.841, 59.888)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

         PathChain Path7 = new PathBuilder(follower)
                .addPath(new BezierLine(new Pose(18.841, 59.888), new Pose(55.000, 15.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(295))
                .build();

         PathChain Path8 = new PathBuilder(follower)
                .addPath(new BezierLine(new Pose(55.000, 15.000), new Pose(44.187, 84.561)))
                .setLinearHeadingInterpolation(Math.toRadians(295), Math.toRadians(180))
                .build();

         PathChain Path9 = new PathBuilder(follower)
                .addPath(new BezierLine(new Pose(44.187, 84.561), new Pose(17.720, 84.112)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

         PathChain Path10 = new PathBuilder(follower)
                .addPath(new BezierLine(new Pose(17.720, 84.112), new Pose(55.000, 15.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(295))
                .build();



        waitForStart();
      //  runPath(follower, Path1,3000);
        runPath(follower, Path2,0);
        runPath(follower, Path3,1000);
        runPath(follower, Path4,3000);
        runPath(follower, Path5,0);
        runPath(follower, Path6,1000);
        runPath(follower, Path7,3000);
        runPath(follower, Path8,0);
        runPath(follower, Path9,1000);
        runPath(follower, Path10,3000);


    }
    private void runPath(Follower follower, PathChain path, long delayAfterMs) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        if (delayAfterMs > 0) sleep(delayAfterMs);
    }

}
