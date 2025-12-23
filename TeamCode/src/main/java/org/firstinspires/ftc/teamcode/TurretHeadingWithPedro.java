package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Turret Heading Control (Pedro)")
public class TurretHeadingWithPedro extends LinearOpMode {

    Servo turret;

    // Adjust these to your servo’s real limits
    final double SERVO_MIN = 0.0;
    final double SERVO_MAX = 1.0;

    // Where we want turret to face (0° = up-field, can change)
    double targetHeadingDeg = 0;

    @Override
    public void runOpMode() {

        // Get follower (this internally starts Pinpoint localization)
        Follower follower = Constants.createFollower(hardwareMap);

        turret = hardwareMap.get(Servo.class, "turret");

        waitForStart();

        while (opModeIsActive()) {

            // --------------------------------------------------------------------
            // 1. GET HEADING FROM PEDRO PATHING
            // --------------------------------------------------------------------
            double robotHeadingRad = follower.getPose().getHeading();
            double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

            // Your robot starts pointed at 90°, so offset
            robotHeadingDeg = robotHeadingDeg - 90;

            // Normalize 0–360°
            robotHeadingDeg = (robotHeadingDeg % 360 + 360) % 360;


            // --------------------------------------------------------------------
            // 2. CALCULATE REQUIRED ANGLE FOR TURRET
            // --------------------------------------------------------------------
            double neededAngle = targetHeadingDeg - robotHeadingDeg;

            neededAngle = (neededAngle % 360 + 360) % 360;


            // --------------------------------------------------------------------
            // 3. MAP ANGLE TO SERVO POSITION 0–1
            // --------------------------------------------------------------------
            double servoPos = SERVO_MIN +
                    (neededAngle / 360.0) * (SERVO_MAX - SERVO_MIN);

            servoPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, servoPos));

            turret.setPosition(servoPos);


            // --------------------------------------------------------------------
            // 4. TELEMETRY
            // --------------------------------------------------------------------
            telemetry.addData("Robot Heading (Deg)", robotHeadingDeg);
            telemetry.addData("Needed Angle", neededAngle);
            telemetry.addData("Servo Position", servoPos);
            telemetry.update();

            // Update Pedro internal localization (required)
            follower.update();
        }
    }
}
