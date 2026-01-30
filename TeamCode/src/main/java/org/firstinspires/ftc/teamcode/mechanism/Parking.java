//package org.firstinspires.ftc.teamcode.mechanism;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class Parking {
//    public Servo leftPark, rightPark;
//
//    public Parking(HardwareMap hw) {
//        leftPark = hw.get(Servo.class, "leftpark");
//        rightPark = hw.get(Servo.class, "rightpark");
//        rightPark.setDirection(Servo.Direction.REVERSE);
//
//        // Initial position
//        leftPark.setPosition(0);
//        rightPark.setPosition(0);
//    }
//
//    // Call this when you want to deploy parking
//    public void deploy() {
//        leftPark.setPosition(1);
//        rightPark.setPosition(1);
//    }
//
//    // Optional: Call this if you want to retract
//    public void retract() {
//        leftPark.setPosition(0);
//        rightPark.setPosition(0);
//    }
//}
