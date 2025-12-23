//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@TeleOp(name = "Two Motor Max Velocity Test", group = "Tests")
//public class MotorMaxVelocityTest extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
//        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
//        motor1.setDirection(F);
//
//
//        // Reset encoders
//        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // Use RUN_WITHOUT_ENCODER to measure raw speed
//        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        telemetry.addLine("Ready to test two motors.");
//        telemetry.addLine("Press PLAY to start.");
//        telemetry.update();
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        // Ramp both motors to full power
//        motor1.setPower(1.0);
//        motor2.setPower(1.0);
//
//        // Allow motors to settle at top speed
//        sleep(1000);
//
//        // Read velocity in ticks/second
//        double velTicks1 = motor1.getVelocity();
//        double velTicks2 = motor2.getVelocity();
//
//        // Convert ticks/sec → RPM
//        double tpr1 = motor1.getMotorType().getTicksPerRev();
//        double tpr2 = motor2.getMotorType().getTicksPerRev();
//
//        double rpm1 = velTicks1 * 60.0 / tpr1;
//        double rpm2 = velTicks2 * 60.0 / tpr2;
//
//        telemetry.addLine("======= TWO MOTOR SPEED RESULTS =======");
//        telemetry.addLine("--- Motor 1 ---");
//        telemetry.addData("Ticks/Sec", velTicks1);
//        telemetry.addData("RPM", rpm1);
//        telemetry.addData("Ticks/Rev", tpr1);
//        telemetry.addData("Theoretical Max RPM", motor1.getMotorType().getMaxRPM());
//
//        telemetry.addLine("\n--- Motor 2 ---");
//        telemetry.addData("Ticks/Sec", velTicks2);
//        telemetry.addData("RPM", rpm2);
//        telemetry.addData("Ticks/Rev", tpr2);
//        telemetry.addData("Theoretical Max RPM", motor2.getMotorType().getMaxRPM());
//
//        telemetry.update();
//
//        // Continuously show data until STOP
//        while (opModeIsActive()) {
//            idle();
//        }
//
//        motor1.setPower(0);
//        motor2.setPower(0);
//    }
//}
