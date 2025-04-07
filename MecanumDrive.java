package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class MecanumDrive {
    // Motor declarations
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Motor power values (public for telemetry)
    public double frontLeftPower;
    public double frontRightPower;
    public double backLeftPower;
    public double backRightPower;
    
    // Power boost factor for extra speed
    private final double POWER_BOOST = 1.0; // Can go up to 1.0 for maximum speed

    // Constructor
    public MecanumDrive(HardwareMap hardwareMap) {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        // Set motor directions
        // Adjust these based on your robot's configuration
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior to FLOAT for less resistance
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set motors to run without encoder for maximum speed
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Method to drive the robot using mecanum drive
    public void drive(double drive, double strafe, double turn) {
        // Apply power curve for more responsive control at lower speeds
        drive = applyPowerCurve(drive);
        strafe = applyPowerCurve(strafe);
        turn = applyPowerCurve(turn);
        
        // Calculate motor powers using mecanum formula
        frontLeftPower = drive + strafe + turn;
        frontRightPower = drive - strafe - turn;
        backLeftPower = drive - strafe + turn;
        backRightPower = drive + strafe - turn;

        // Normalize motor powers if any exceeds 1.0
        double maxPower = Math.max(
            Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        
    
        
        
        // Apply power boost (can make the robot faster)
        frontLeftPower *= POWER_BOOST;
        frontRightPower *= POWER_BOOST;
        backLeftPower *= POWER_BOOST;
        backRightPower *= POWER_BOOST;

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(-backLeftPower);
        backRight.setPower(backRightPower);
    }
    
    // Apply a power curve to make controls more responsive
    private double applyPowerCurve(double input) {
        // Square the input while preserving the sign for a more responsive feel
        return input * Math.abs(input);
    }

    // Reset encoders
    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
