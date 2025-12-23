package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanism.AutoShoot;
import org.firstinspires.ftc.teamcode.mechanism.TurretTracker;

@TeleOp(name = "Drive + Smart Shooter")
public class TeleOpDrive extends LinearOpMode {

    DcMotor FL, BL, FR, BR;
    DcMotor intake;
    CRServo gripwheel, turretRight,turretLeft;

    AutoShoot shooter;
    TurretTracker turret;

    Limelight3A limelight;

    @Override
    public void runOpMode() {

        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        intake = hardwareMap.get(DcMotor.class, "intake");
        gripwheel = hardwareMap.get(CRServo.class, "gripwheel");
        turretLeft = hardwareMap.get(CRServo.class, "turretLeft");
        turretRight = hardwareMap.get(CRServo.class, "turretRight");

        shooter = new AutoShoot(hardwareMap, telemetry);
        turret  = new TurretTracker(hardwareMap, telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            turret.update();

            LLResult r = limelight.getLatestResult();
            boolean hasTarget = r != null && r.isValid();

            if (gamepad2.right_bumper && hasTarget && !shooter.isBusy()) {
                shooter.shootThreeBallsFromLimelight();
            }

            shooter.update();

            if (!shooter.isBusy()) {
                if (gamepad2.left_trigger > 0.1) {
                    intake.setPower(1);
                    gripwheel.setPower(0);

                } else if (gamepad2.right_trigger > 0.1) {
                    intake.setPower(-1);
                    gripwheel.setPower(1);
                } else {
                    intake.setPower(0);
                    gripwheel.setPower(0);
                }

                if (gamepad2.dpad_left) {
                    turret.disable();
                    turretLeft.setPower(-0.7);
                    turretRight.setPower(-0.7);
                } else if (gamepad2.dpad_right) {
                    turret.disable();
                    turretLeft.setPower(0.7);
                    turretRight.setPower(0.7);
                } else {
                    turretLeft.setPower(0);
                    turretRight.setPower(0);
                    turret.enable();
                }

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                FL.setPower((y + x + rx) / d);
                BL.setPower((y - x + rx) / d);
                FR.setPower((y - x - rx) / d);
                BR.setPower((y + x - rx) / d);

                telemetry.addData("Distance (in)", "%.2f", shooter.getDistanceToTarget());
                telemetry.addData("Target Vel", "%.0f", shooter.getTargetVelocity());
                telemetry.addData("Actual Vel", "%.0f", shooter.getActualVelocity());
                telemetry.addData("Left Vel", "%.0f", shooter.getLeftVelocity());
                telemetry.addData("Right Vel", "%.0f", shooter.getRightVelocity());
                telemetry.update();
            }
        }
    }
}
