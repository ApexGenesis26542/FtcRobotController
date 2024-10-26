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

    // Define motor objects for each wheel and viper slides
    private DcMotor frontLeft, frontRight, backLeft, backRight, leftViper, rightViper;

    @Override
    public void runOpMode() {
        // Initialize the motors for chassis and viper slides
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        leftViper = hardwareMap.get(DcMotor.class, "leftviper");
        rightViper = hardwareMap.get(DcMotor.class, "rightviper");

        // Set zero power behavior for each motor to BRAKE, which helps hold the position when no power is applied
        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight, leftViper, rightViper};
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Set motor directions to ensure correct movement of the robot
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
        double driveMultiplier = 0.60;
        double strafeMultiplier = 0.60;
        double turnMultiplier = 5.0 / 12.0;

        // Check if turbo mode is enabled (using bumpers)
        // Turbo mode increases the speed of the robot
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            driveMultiplier = 1.0;
            strafeMultiplier = 1.0;
            turnMultiplier = 0.89;
        }

        // Get joystick input values with exponential scaling for finer control at low speeds
        // Applying cubic transformation to provide smoother control at lower speeds
        double drive = -Math.pow(gamepad1.left_stick_y, 3) * driveMultiplier;  // Forward/Backward motion
        double strafe = -Math.pow(gamepad1.left_stick_x, 3) * strafeMultiplier; // Left/Right strafing motion
        double turn = -Math.pow(gamepad1.right_stick_x, 3) * turnMultiplier;    // Rotational (turning) motion

        // Deadzone adjustments for joysticks to prevent drift
        if (Math.abs(drive) < 0.05) drive = 0;
        if (Math.abs(strafe) < 0.05) strafe = 0;
        if (Math.abs(turn) < 0.05) turn = 0;

        // Precision mode for fine adjustments using 'A' button
        // Precision mode reduces the speed multipliers to make small adjustments easier
        if (gamepad1.a) {
            driveMultiplier *= 0.4;
            strafeMultiplier *= 0.4;
            turnMultiplier *= 0.4;
        }

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
        // Find the maximum power value and divide all by that value if it exceeds 1.0
        double max = Math.max(1.0, Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

        leftFrontPower /= max;
        rightFrontPower /= max;
        leftBackPower /= max;
        rightBackPower /= max;

        // Ramp up power gradually to avoid jerky movements
        // This helps in reducing sudden jumps in speed, providing smoother control
        frontLeft.setPower(gradualPower(frontLeft.getPower(), leftFrontPower, 0.1));
        frontRight.setPower(gradualPower(frontRight.getPower(), rightFrontPower, 0.1));
        backLeft.setPower(gradualPower(backLeft.getPower(), leftBackPower, 0.1));
        backRight.setPower(gradualPower(backRight.getPower(), rightBackPower, 0.1));
    }

    /**
     * Gradually adjust power to avoid sudden changes.
     *
     * @param currentPower Current motor power
     * @param targetPower  Target motor power
     * @param increment    The maximum allowed change in power per cycle
     * @return Adjusted power value
     */
    private double gradualPower(double currentPower, double targetPower, double increment) {
        // Increment or decrement power in small steps to avoid sudden jerks
        if (currentPower < targetPower) {
            return Math.min(currentPower + increment, targetPower);
        } else if (currentPower > targetPower) {
            return Math.max(currentPower - increment, targetPower);
        }
        return targetPower;
    }

    /**
     * Handle viper slide movement using dpad inputs.
     */
    private void handleViperSlides() {
        double viperPower = 0.0; // Default power is zero to prevent unintended movement
        if (gamepad1.dpad_up) {
            viperPower = 0.3;  // Move viper slides upwards
        } else if (gamepad1.dpad_down) {
            viperPower = -0.5; // Move viper slides downwards
        } else {
            viperPower = 0.05; // Hold position to prevent sliding due to gravity
        }

        // Turbo mode increases viper slide power to 1.0 for faster movement
        if ((gamepad1.right_bumper || gamepad1.left_bumper) && (gamepad1.dpad_up || gamepad1.dpad_down)) {
            viperPower = Math.signum(viperPower);
        }

        // Set power for both viper slide motors
        leftViper.setPower(viperPower);
        rightViper.setPower(viperPower);

        // Precision mode for viper slides using 'X' button
        // Reduces the power to half for finer control of slide movement
        if (gamepad1.x) {
            leftViper.setPower(viperPower * 0.5);
            rightViper.setPower(viperPower * 0.5);
        }
    }
}
