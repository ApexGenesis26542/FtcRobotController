package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Combined Chassis and Viper Slide class for FTC robot teleop mode.
 */
@TeleOp
public class FullDrive extends LinearOpMode {

    // Define motor objects for each wheel
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    // Define motor objects for the viper slides
    private DcMotor leftViper;
    private DcMotor rightViper;

    @Override
    public void runOpMode() {
        // Initialize the motors for chassis
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");

        // Initialize the motors for viper slides
        leftViper = hardwareMap.get(DcMotor.class, "leftviper");
        rightViper = hardwareMap.get(DcMotor.class, "rightviper");

        // Set zero power behavior for each motor to BRAKE, which helps hold the position when no power is applied
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor directions
        // Reversing the direction of the right-side motors to ensure correct movement
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Display message to user to indicate that the robot is ready to start
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Wait for user to start op mode
        waitForStart();

        while (opModeIsActive()) {
            // Handle chassis movement using joysticks
            handleChassisMovement();
            // Handle viper slide movement using dpad
            handleViperSlides();
            // Wait for 10ms before updating again to prevent rapid looping
            sleep(10);
        }
    }

    /**
     * Handle chassis movement using joystick inputs.
     */
    private void handleChassisMovement() {
        // Define multipliers for drive, strafe, and turn rates
        double o_drive_multiplier = 0.60; // Normal drive speed multiplier
        double o_strafe_multiplier = 0.60; // Normal strafe speed multiplier
        double o_turn_multiplier = (double) 5 / 12; // Normal turn speed multiplier
        double n_drive_multiplier = o_drive_multiplier;
        double n_strafe_multiplier = o_strafe_multiplier;
        double n_turn_multiplier = o_turn_multiplier;

        // Check if turbo mode is enabled (using bumpers)
        // Turbo mode increases the speed multipliers for faster movement
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            n_drive_multiplier = 1;
            n_strafe_multiplier = 1;
            n_turn_multiplier = 0.89;
        }

        // Get joystick input values
        // The left stick controls forward/backward and strafing
        // The right stick controls turning
        double drive = -gamepad1.left_stick_y * n_drive_multiplier;  // Forward/Backward (Y-axis) using left stick
        double strafe = -gamepad1.left_stick_x * n_strafe_multiplier;  // Strafing (X-axis) using left stick
        double turn = -gamepad1.right_stick_x * n_turn_multiplier;  // Turning (X-axis) using right stick

        // Display telemetry data for debugging and feedback
        telemetry.addData("Details", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        telemetry.update();

        // Move the robot using the calculated motor powers
        moveRobot(strafe, drive, turn);
    }

    /**
     * Move the robot using the calculated motor powers.
     *
     * @param x   Desired strafe motion (-1 to +1)
     * @param y   Desired forward/backward motion (-1 to +1)
     * @param yaw Desired turning motion (-1 to +1)
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers based on the desired motions
        double leftFrontPower = y + x - yaw;
        double rightFrontPower = y - x + yaw;
        double leftBackPower = y - x - yaw;
        double rightBackPower = y + x + yaw;

        // Normalize wheel powers to ensure they do not exceed 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send the calculated power values to the wheels
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);

        // Display motor powers for debugging and monitoring purposes
        telemetry.addData("Motor Powers", "LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.update();
    }

    /**
     * Handle viper slide movement using dpad inputs.
     */
    private void handleViperSlides() {
        if (gamepad1.dpad_up) {
            // Move viper slides upwards with reduced power for controlled movement
            leftViper.setPower(0.3);
            rightViper.setPower(0.3);
        } else if (gamepad1.dpad_down) {
            // Move viper slides downwards with more power
            leftViper.setPower(-0.5);
            rightViper.setPower(-0.5);
        } else {
            // Apply a small power to hold the position and prevent sagging due to gravity
            leftViper.setPower(0.05);
            rightViper.setPower(0.05);
        }

        // Display telemetry data for viper slides to monitor their status
        telemetry.addData("Left Viper Power", leftViper.getPower());
        telemetry.addData("Right Viper Power", rightViper.getPower());
        telemetry.update();
    }
}