package org.firstinspires.ftc.teamcode.mechanism;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp (name = "align")

public class Vision extends LinearOpMode {

    DcMotor RF,LR,RR,LF;
    Limelight3A limelight;
    double kp = 0.02;
    double minPower = 0.1;
    double threshold = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        LF = hardwareMap.get(DcMotor.class,"LF");
        LR = hardwareMap.get(DcMotor.class,"LR");
        RF = hardwareMap.get(DcMotor.class,"RF");
        RR = hardwareMap.get(DcMotor.class,"RR");

        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LR.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RR.setDirection(DcMotorSimple.Direction.FORWARD);

        limelight.pipelineSwitch(6);
        limelight.start();
        waitForStart();

        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double turnPower  = kp * tx;

                if (Math.abs(turnPower)<minPower && tx>threshold){
                    turnPower = Math.signum(turnPower) * minPower;
                }
                if (Math.abs(tx) <= threshold) {
                   LF.setPower(0);
                   LR.setPower(0);
                   RF.setPower(0);
                   RR.setPower(0);

                } else {
                    LF.setPower(turnPower);
                    LR.setPower(turnPower);
                    RF.setPower(-turnPower);
                    RR.setPower(-turnPower);

                }


            } else {
                LF.setPower(0);
                LR.setPower(0);
                RF.setPower(0);
                RR.setPower(0);

            }

            telemetry.update();
        }

        limelight.stop();
            }

        }



