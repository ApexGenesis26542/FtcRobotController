package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Combined Chassis and Twin Tower class for FTC robot teleop mode.
 */
@TeleOp
public class FullDrive extends LinearOpMode {

    // Define motor objects for each wheel and twin tower
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx twinTowerMotor;
    private Servo geckowheel;
    private boolean isIntakeToggled = false;
    private boolean lastBState = false;

    @Override
    public void runOpMode() {
        // Initialize the motors for chassis and twin tower
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        twinTowerMotor = hardwareMap.get(DcMotorEx.class, "twintower");
        geckowheel = hardwareMap.get(Servo.class, "geckowheel");
        twinTowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        twinTowerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Set the motor to RUN_USING_ENCODER when not moving to a specific position
        // Set zero power behavior for each motor to BRAKE, which helps hold the position when no power is applied
        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight, twinTowerMotor};
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Revert motor directions to ensure correct movement of the robot
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
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
            // Handle twin tower movement using triggers
            handleTwinTower();
            // Handle Intake/Outtake
            handleIntakeOutake();

            // Update the telemetry with current twin tower motor position
            telemetry.addData("Twin Tower Motor Position", twinTowerMotor.getCurrentPosition());
            telemetry.addData("Front Left Motor Power", frontLeft.getPower());
            telemetry.addData("Front Right Motor Power", frontRight.getPower());
            telemetry.addData("Back Left Motor Power", backLeft.getPower());
            telemetry.addData("Back Right Motor Power", backRight.getPower());
            telemetry.addData("Twin Tower Motor Target Position", twinTowerMotor.getTargetPosition());
            telemetry.addData("Gecko Wheel Servo Position", geckowheel.getPosition());
            telemetry.addData("Twin Tower Motor Mode", twinTowerMotor.getMode());
            telemetry.addData("Front Left Motor ZeroPowerBehavior", frontLeft.getZeroPowerBehavior());
            telemetry.addData("Front Right Motor ZeroPowerBehavior", frontRight.getZeroPowerBehavior());
            telemetry.addData("Back Left Motor ZeroPowerBehavior", backLeft.getZeroPowerBehavior());
            telemetry.addData("Back Right Motor ZeroPowerBehavior", backRight.getZeroPowerBehavior());
            telemetry.addData("Twin Tower Motor ZeroPowerBehavior", twinTowerMotor.getZeroPowerBehavior());
            telemetry.update();
            // Non-blocking delay to improve responsiveness
            idle();
        }
    }

    /**
     * Handle chassis movement using joystick inputs.
     */
    private void handleChassisMovement() {
        // Define multipliers for drive, strafe, and turn rates
        double driveMultiplier = 0.85;
        double strafeMultiplier = 0.85;
        double turnMultiplier = 0.75;

        // Check if turbo mode is enabled (using bumpers)
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            driveMultiplier = 1.0; // Set drive multiplier to full power for turbo mode
            strafeMultiplier = 1.0; // Set strafe multiplier to full power for turbo mode
            turnMultiplier = 0.90;  // Set turn multiplier to higher value for faster turns
        }

        // Get joystick input values with exponential scaling for finer control at low speeds
        double drive = -Math.pow(gamepad1.left_stick_y, 3) * driveMultiplier;  // Forward/Backward motion
        double strafe = gamepad1.left_stick_x * strafeMultiplier; // Left/Right strafing motion
        double turn = gamepad1.right_stick_x * turnMultiplier; // Curvature drive for smoother turning

        // Deadzone adjustments for joysticks to prevent drift
        if (Math.abs(drive) < 0.05) drive = 0; // Ignore small inputs for drive
        if (Math.abs(strafe) < 0.05) strafe = 0; // Ignore small inputs for strafe
        if (Math.abs(turn) < 0.05) turn = 0; // Ignore small inputs for turn

        // Precision mode for fine adjustments using 'A' button
        if (gamepad1.a) {
            driveMultiplier *= 0.4; // Reduce drive multiplier for precision control
            strafeMultiplier *= 0.4; // Reduce strafe multiplier for precision control
            turnMultiplier *= 0.4;   // Reduce turn multiplier for precision control
        }

        // Move the robot using the calculated motor powers
        moveRobot(strafe, drive, turn);

        // Telemetry for chassis movement
        telemetry.addData("Drive Multiplier", driveMultiplier);
        telemetry.addData("Strafe Multiplier", strafeMultiplier);
        telemetry.addData("Turn Multiplier", turnMultiplier);
        telemetry.addData("Drive Power", drive);
        telemetry.addData("Strafe Power", strafe);
        telemetry.addData("Turn Power", turn);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.addData("Right Bumper Pressed", gamepad1.right_bumper);
        telemetry.addData("Left Bumper Pressed", gamepad1.left_bumper);
        telemetry.addData("A Button Pressed", gamepad1.a);
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
        double leftFrontPower = y + x + yaw;
        double rightFrontPower = y - x - yaw;
        double leftBackPower = y - x + yaw;
        double rightBackPower = y + x - yaw;

        // Normalize wheel powers to ensure they do not exceed 1.0
        double max = Math.max(1.0, Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

        leftFrontPower /= max;
        rightFrontPower /= max;
        leftBackPower /= max;
        rightBackPower /= max;

        // Set power to each motor directly without ramp-up to avoid inconsistent timing issues
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);

        // Telemetry for wheel powers
        telemetry.addData("Left Front Power (Target)", leftFrontPower);
        telemetry.addData("Right Front Power (Target)", rightFrontPower);
        telemetry.addData("Left Back Power (Target)", leftBackPower);
        telemetry.addData("Right Back Power (Target)", rightBackPower);
        telemetry.addData("Left Front Motor Actual Power", frontLeft.getPower());
        telemetry.addData("Right Front Motor Actual Power", frontRight.getPower());
        telemetry.addData("Left Back Motor Actual Power", backLeft.getPower());
        telemetry.addData("Right Back Motor Actual Power", backRight.getPower());
    }

    /**
     * Handle twin tower movement using triggers.
     */
    private void handleTwinTower() {
        double twinTowerMotorMultiplier = 0.7;
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            twinTowerMotorMultiplier = 1;
        }
        if (gamepad1.a) {
            twinTowerMotorMultiplier *= 0.4;
        }
        // Use right trigger to move the twin tower motor forward
            double rtpower = gamepad1.right_trigger * twinTowerMotorMultiplier;
            double ltpower = gamepad1.left_trigger * twinTowerMotorMultiplier;
            if (gamepad1.right_trigger > 0) {
                int targetPosition = twinTowerMotor.getCurrentPosition() + (int) (gamepad1.right_trigger * 1993.6); // Calculate target position based on trigger input and encoder PPR
                twinTowerMotor.setTargetPosition(targetPosition); // Set the target position for the motor
                twinTowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set the motor to run to the target position
                twinTowerMotor.setPower(rtpower); // Set power to move towards the target position
            }
            // Use left trigger to move the twin tower motor backward
            else if (gamepad1.left_trigger > 0) {
                int targetPosition = twinTowerMotor.getCurrentPosition() - (int) (gamepad1.left_trigger * 1993.6); // Calculate target position based on trigger input and encoder PPR
                twinTowerMotor.setTargetPosition(targetPosition); // Set the target position for the motor
                twinTowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set the motor to run to the target position
                twinTowerMotor.setPower(ltpower); // Set power to move towards the target position
            } else {
                twinTowerMotor.setPower(1); // Stop the motor if no trigger is pressed
            }

        // Telemetry for twin tower motor power
        telemetry.addData("Twin Tower Motor Power", twinTowerMotor.getPower());
        telemetry.addData("Twin Tower Motor Multiplier", twinTowerMotorMultiplier);
        telemetry.addData("Right Trigger Power", rtpower);
        telemetry.addData("Left Trigger Power", ltpower);
        telemetry.addData("Twin Tower Motor Current Position", twinTowerMotor.getCurrentPosition());
        telemetry.addData("Twin Tower Motor Target Position", twinTowerMotor.getTargetPosition());
        telemetry.addData("Twin Tower Motor Mode", twinTowerMotor.getMode());
        telemetry.addData("Right Bumper Pressed", gamepad1.right_bumper);
        telemetry.addData("Left Bumper Pressed", gamepad1.left_bumper);
        telemetry.addData("A Button Pressed", gamepad1.a);
        telemetry.addData("Twin Tower Motor ZeroPowerBehavior", twinTowerMotor.getZeroPowerBehavior());
        telemetry.addData("Twin Tower Motor Direction", twinTowerMotor.getDirection());
    }

    /**
     * Handle intake/outtake mechanism using dpad inputs.
     */
    private void handleIntakeOutake() {
        if (gamepad1.b && !lastBState) {
            isIntakeToggled = !isIntakeToggled;
            lastBState = true;
        } else if (!gamepad1.b) {
            lastBState = false;
        }

        if (isIntakeToggled) {
            geckowheel.setPosition(1);
        } else {
            geckowheel.setPosition(0);
        }
        // Telemetry for intake/outtake mechanism
        telemetry.addData("Intake/Outake Mechanism Toggled", isIntakeToggled);
        telemetry.addData("Gecko Wheel Position", geckowheel.getPosition());
        telemetry.addData("Intake Toggled", isIntakeToggled);
        telemetry.addData("Dpad Up Pressed", gamepad1.dpad_up);
        telemetry.addData("Dpad Down Pressed", gamepad1.dpad_down);
        telemetry.addData("B Button Pressed", gamepad1.b);
        telemetry.addData("Gecko Wheel Direction", geckowheel.getDirection());
    }
}
