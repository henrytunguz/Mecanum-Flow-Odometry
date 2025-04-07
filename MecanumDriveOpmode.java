package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum Drive", group="Linear Opmode")
public class MecanumDriveOpmode extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive mecanumDrive;
    
    // Speed control variables
    private double speedMultiplier = 1.0; // Default to full speed
    private boolean isFastMode = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the mecanum drive
        mecanumDrive = new MecanumDrive(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Toggle speed mode with B button
            if (gamepad1.right_bumper && !wasPressed) {
                isFastMode = !isFastMode;
                speedMultiplier = isFastMode ? 1.0 : 0.5;
                wasPressed = true;
            } else if (!gamepad1.right_bumper) {
                wasPressed = false;
            }
            
            // Boost mode - hold Y for temporary max speed
            if (gamepad1.left_bumper){
                speedMultiplier = 1.0;
            } else if (!isFastMode && !gamepad1.left_bumper) {
                speedMultiplier = 0.5;
            }

            // Get gamepad inputs
            double drive = -gamepad1.left_stick_y;  // Forward/backward
            double strafe = gamepad1.left_stick_x;  // Left/right
            double turn = gamepad1.right_stick_x;   // Rotation

            // Apply deadzone to prevent drift
            drive = applyDeadzone(drive);
            strafe = applyDeadzone(strafe);
            turn = applyDeadzone(turn);
            
            // Apply speed multiplier
            drive *= speedMultiplier;
            strafe *= speedMultiplier;
            turn *= speedMultiplier;

            // Drive the robot
            mecanumDrive.drive(drive, strafe, turn);

            // Show the elapsed game time and wheel powers
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed Mode", isFastMode ? "FAST" : "NORMAL");
            telemetry.addData("Drive", "drive (%.2f), strafe (%.2f), turn (%.2f)", drive, strafe, turn);
            telemetry.addData("Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", 
                mecanumDrive.frontLeftPower, mecanumDrive.frontRightPower, 
                mecanumDrive.backLeftPower, mecanumDrive.backRightPower);
            telemetry.update();
        }
    }

    private boolean wasPressed = false;
    
    private double applyDeadzone(double value) {
        // Apply a smaller deadzone for more responsive control
        final double DEADZONE = 0.03;
        return Math.abs(value) < DEADZONE ? 0 : value;
    }
}
