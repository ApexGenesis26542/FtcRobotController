package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * ChassisTest class for FTC robot teleop mode.
 */
@TeleOp
public class ChassisTest extends LinearOpMode {

    // Define motor objects for each wheel
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {
        // Initialize the motors after hardwareMap is available
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");

        // Initialize variables for desired motor powers
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        // Set zero power behavior for each motor
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor directions
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Display message to user
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Wait for user to start op mode
        waitForStart();

        while (opModeIsActive()) {
            // Display message to user
            telemetry.addData("\n>", "Drive using joysticks\n");

            // Define multipliers for drive, strafe, and turn rates
            double o_drive_multiplier = 0.60;
            double o_strafe_multiplier = 0.60;
            double o_turn_multiplier = (double) 5 / 12;
            double n_drive_multiplier = o_drive_multiplier;
            double n_strafe_multiplier = o_strafe_multiplier;
            double n_turn_multiplier = o_turn_multiplier;

            // Check if turbo mode is enabled
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                n_drive_multiplier = 1;
                n_strafe_multiplier = 1;
                n_turn_multiplier = 0.89;
            } else {
                n_drive_multiplier = o_drive_multiplier;
                n_strafe_multiplier = o_strafe_multiplier;
                n_turn_multiplier = o_turn_multiplier;
            }

            // Get joystick input values
            drive = -gamepad1.left_stick_y * n_drive_multiplier;  // Reduce drive rate to 75%.
            strafe = -gamepad1.left_stick_x * n_strafe_multiplier;  // Reduce strafe rate to 75%.
            turn = gamepad1.right_stick_x * n_turn_multiplier;  // Reduce turn rate to 66%.

            // Display telemetry data
            telemetry.addData("Details", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.update();

            // Move the robot using the calculated motor powers
            moveRobot(-turn, drive, strafe);

            // Wait for 10ms before updating again
            sleep(10);
        }
    }

    /**
     * Move the robot using the calculated motor powers.
     *
     * @param x  Desired x-axis motion (-1 to +1)
     * @param y  Desired y-axis motion (-1 to +1)
     * @param yaw Desired yaw motion (-1 to +1)
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x + y - yaw;
        double rightFrontPower = x - y + yaw;
        double leftBackPower = x - y - yaw;
        double rightBackPower = x + y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);

        // Display motor powers
        telemetry.addData("Motor Powers", "LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.update();
    }
}
