package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Combined Chassis and Twin Tower class for FTC robot teleop mode.
 * The class handles movement of the robot chassis, as well as the operation of a twin tower mechanism.
 */
@TeleOp
@Config
public class FullDrive extends LinearOpMode {

    // Static variables to define positions for picking and dropping objects
    public static double PICKPOSITION = -3137;
    public static double DROPPOSITION = -2260;

    // Declare motor and sensor objects
    private DcMotor frontLeft, frontRight, backLeft, backRight, slide;
    private DcMotorEx twinTowerMotor;
    private Servo angler;
    private DistanceSensor distanceSensor;

    public static boolean approximatelyEqual(double a, double b, double tolerance) {
        return Math.abs(a - b) <= tolerance;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors and sensors from the hardware map
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        twinTowerMotor = hardwareMap.get(DcMotorEx.class, "twintower");
        slide = hardwareMap.get(DcMotor.class, "slides");
        angler = hardwareMap.get(Servo.class, "angler");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        telemetry.addData("Initialization", "Motors and Sensors Initialized");

        // Reset encoders for twin tower motor and slide
        twinTowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        twinTowerMotor.setTargetPosition(0);
        twinTowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        twinTowerMotor.setPower(1);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);

        // Set motors to brake when power is zero
        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight, twinTowerMotor, slide};
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Motor", "%s set to BRAKE mode", motor.getDeviceName());
        }

        // Set motor directions to ensure correct movement
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Initialization", "Motor Directions Set");

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        // Main loop to handle robot operation during teleop
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            handleRobotCentricMovement();
            handleTwinTower();
            handleIntakeOutake();
            telemetry.update();
            idle();
        }
    }

    private void handleRobotCentricMovement() {
        double driveMultiplier = 0.85;
        double strafeMultiplier = 0.85;
        double turnMultiplier = 0.75;

        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            driveMultiplier = 1.0;
            strafeMultiplier = 1.0;
            turnMultiplier = 0.90;
        }

        if (gamepad1.a) {
            driveMultiplier *= 0.4;
            strafeMultiplier *= 0.4;
            turnMultiplier *= 0.4;
        }

        double x1 = gamepad1.left_stick_x * strafeMultiplier;
        double y1 = -gamepad1.left_stick_y * driveMultiplier;
        double turn = gamepad1.right_stick_x * turnMultiplier;

        double leftFrontPower = y1 + x1 + turn;
        double rightFrontPower = y1 - x1 - turn;
        double leftBackPower = y1 - x1 + turn;
        double rightBackPower = y1 + x1 - turn;

        double max = Math.max(1.0, Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));
        leftFrontPower /= max;
        rightFrontPower /= max;
        leftBackPower /= max;
        rightBackPower /= max;

        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);

        telemetry.addData("Motor Powers (Robot-Centric)", "LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private void handleTwinTower() {
        double twinTowerMotorMultiplier = 0.7;
        if (gamepad1.right_bumper || gamepad1.left_bumper) twinTowerMotorMultiplier = 1.0;
        if (gamepad1.a) twinTowerMotorMultiplier *= 0.4;

        double ttrtpower = gamepad1.right_trigger * twinTowerMotorMultiplier;
        double ttltpower = gamepad1.left_trigger * twinTowerMotorMultiplier;

        telemetry.addData("Twin Tower Inputs", "Right Trigger: %.2f, Left Trigger: %.2f", gamepad1.right_trigger, gamepad1.left_trigger);

        if (gamepad1.right_trigger > 0) {
            int targetPosition = twinTowerMotor.getCurrentPosition() + (int) (gamepad1.right_trigger * 0.7 * 1993.6);
            twinTowerMotor.setTargetPosition(targetPosition);
            twinTowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            twinTowerMotor.setPower(ttrtpower);
            telemetry.addData("Twin Tower", "Target Position: %d, Power: %.2f", targetPosition, ttrtpower);
        } else if (gamepad1.left_trigger > 0) {
            int targetPosition = twinTowerMotor.getCurrentPosition() - (int) (gamepad1.left_trigger * 0.7 * 1993.6);
            twinTowerMotor.setTargetPosition(Math.max(targetPosition, 0));
            twinTowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            twinTowerMotor.setPower(ttltpower);
            telemetry.addData("Twin Tower", "Target Position: %d, Power: %.2f", targetPosition, ttltpower);
        } else {
            twinTowerMotor.setPower(1);
            telemetry.addData("Twin Tower", "Holding Position");
        }

        if (gamepad1.x) {
            twinTowerMotor.setTargetPosition((int) PICKPOSITION);
            twinTowerMotor.setPower(1);
            telemetry.addData("Twin Tower Preset", "Picking Position");
        }
        if (gamepad1.y) {
            twinTowerMotor.setTargetPosition((int) DROPPOSITION);
            twinTowerMotor.setPower(1);
            telemetry.addData("Twin Tower Preset", "Dropping Position");
        }

        telemetry.addData("Twin Tower Motor Power", twinTowerMotor.getPower());
        telemetry.addData("Twin Tower Motor Current Position", twinTowerMotor.getCurrentPosition());
    }

    public boolean isDistanceInRange(double lowerBound, double upperBound) {
        double currentDistance = distanceSensor.getDistance(DistanceUnit.CM);
        return currentDistance >= lowerBound && currentDistance <= upperBound;
    }

    private void handleIntakeOutake() {

        if (gamepad1.dpad_up) {
            slide.setTargetPosition(700);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            angler.setPosition(0.375);
            telemetry.addData("Slide", "Moving to Top Chamber Position");

        }

        if (approximatelyEqual(slide.getCurrentPosition(), 700, 5) && isDistanceInRange(5.5, 6)) {
            angler.setDirection(Servo.Direction.FORWARD);
            angler.setPosition(0.25);
            telemetry.addData("Angler", "Adjusted to Position 0.25");
        }
        if (slide.getCurrentPosition() > 650) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance Sensor", "Distance: %.2f cm", distance);

            if (distance < 5.5) {
                telemetry.addData("Distance", "Too Close. Move Back: %.2f cm", 5.5 - distance);
            } else if (distance > 6) {
                telemetry.addData("Distance", "Too Far. Move Forward: %.2f cm", distance - 6);
            } else {
                telemetry.addData("Distance", "Correct Distance. Proceed");
            }

        }
        if (gamepad1.dpad_down) {
            angler.setDirection(Servo.Direction.FORWARD);
            angler.setPosition(0);
            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            telemetry.addData("Slide", "Reset to Initial Position");
            telemetry.addData("Angler", "Reset to Initial Position");
        }
        if (gamepad1.a) {
            angler.setDirection(Servo.Direction.REVERSE);
            angler.setPosition(0.5);
            telemetry.addData("Angler", "Adjusted to Position 0.5");
        }
    }
}
/*
  Cool things that I should implement:
  TODO: Implement Bulk Reading
  TODO: Make it so that the claw automatically closes when it detects a sample using a distance sensor
  TODO: Switch to Pedro Pathing
  TODO: Consider Using FTCLib
  TODO: Switch to finite state/state machine system
  TODO: place control and expansion hubs in the twin towers
  TODO: Use TOF Distance sensosr(s) to detect when driver has reached correct distance from submersible wall to hang the specimen and same for the perimeter.
  TODO: Do Autonomous
  TODO: Use the "mode" button on the controller to be able to switch between robot centric and field centric driving modes
 */