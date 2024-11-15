package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Combined Chassis and Twin Tower class for FTC robot teleop mode.
 */
@TeleOp
public class FullDrive extends LinearOpMode {

    // Define motor objects for each wheel and twin tower
    private DcMotor frontLeft, frontRight, backLeft, backRight, twinTowerMotor;
    private Servo rotator, geckowheel, angler;

    @Override
    public void runOpMode() {
        // Initialize the motors for chassis and twin tower
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        twinTowerMotor = hardwareMap.get(DcMotor.class, "twintower");
        rotator = hardwareMap.get(Servo.class, "rotator");
        geckowheel = hardwareMap.get(Servo.class, "geckowheel");
        angler = hardwareMap.get(Servo.class, "angler");
        angler.setDirection(Servo.Direction.REVERSE);

        // Set zero power behavior for each motor to BRAKE, which helps hold the position when no power is applied
        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight, twinTowerMotor};
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Set motor directions to ensure correct movement of the robot
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse direction for right side motors
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse direction for left side motors
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

            // Update the telemetry with current chassis and twin tower positions
            telemetry.addData("Chassis Left Motor Position", frontLeft.getCurrentPosition());
            telemetry.addData("Chassis Right Motor Position", frontRight.getCurrentPosition());
            telemetry.addData("Twin Tower Motor Position", twinTowerMotor.getCurrentPosition());
            telemetry.update();

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
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            driveMultiplier = 1.0; // Set drive multiplier to full power for turbo mode
            strafeMultiplier = 1.0; // Set strafe multiplier to full power for turbo mode
            turnMultiplier = 0.89;  // Set turn multiplier to higher value for faster turns
        }

        // Get joystick input values with exponential scaling for finer control at low speeds
        double drive = -Math.pow(gamepad1.left_stick_y, 3) * driveMultiplier;  // Forward/Backward motion
        double strafe = -Math.pow(gamepad1.left_stick_x, 3) * strafeMultiplier; // Left/Right strafing motion
        double turn = -Math.pow(gamepad1.right_stick_x, 3) * turnMultiplier;    // Rotational (turning) motion

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
        telemetry.addData("Drive Power", drive);
        telemetry.addData("Strafe Power", strafe);
        telemetry.addData("Turn Power", turn);
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
        double max = Math.max(1.0, Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

        leftFrontPower /= max;
        rightFrontPower /= max;
        leftBackPower /= max;
        rightBackPower /= max;

        // Ramp up power gradually to avoid jerky movements
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
        // Increment or decrement power gradually to avoid sudden jumps
        if (currentPower < targetPower) {
            return Math.min(currentPower + increment, targetPower);
        } else if (currentPower > targetPower) {
            return Math.max(currentPower - increment, targetPower);
        }
        return targetPower;
    }

    /**
     * Handle twin tower movement using triggers.
     */
    private void handleTwinTower() {
        // Use right trigger to move the twin tower motor forward
        if (gamepad1.right_trigger > 0) {
            twinTowerMotor.setPower(gamepad1.right_trigger / 1.5); // Scale down the power for smooth control
            // Use left trigger to move the twin tower motor backward
        } else if (gamepad1.left_trigger > 0) {
            twinTowerMotor.setPower(-gamepad1.left_trigger / 2); // Scale down the power for smooth control
        } else {
            twinTowerMotor.setPower(0); // Stop the motor if no trigger is pressed
        }

        // Telemetry for twin tower motor power
        telemetry.addData("Twin Tower Motor Power", twinTowerMotor.getPower());
    }

    /**
     * Handle intake/outtake mechanism using dpad inputs.
     */
    private void handleIntakeOutake() {
        if (gamepad1.dpad_up){
            rotator.setPosition((double) 180/300);
        }
        if (gamepad1.dpad_down){
            rotator.setPosition((double) 0/300);
        }
        if (gamepad1.a){
            rotator.setPosition(0);
            angler.setPosition(0);
        }
        if (gamepad1.dpad_left){
            geckowheel.setPosition(1);
        }
        if (gamepad1.dpad_right){
            geckowheel.setPosition(-1);
        }
        if (gamepad1.x){
            angler.setPosition((double) 90/300);
        }
        if (gamepad1.y){
            angler.setPosition(0);
        }
    }
}
