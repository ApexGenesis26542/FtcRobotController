package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Combined Chassis and Twin Tower class for FTC robot teleop mode.
 * The class handles movement of the robot chassis, as well as the operation of a twin tower mechanism.
 */
@TeleOp
@Config
public class FullDrive extends LinearOpMode {

    // Static variables to define positions for picking and dropping objects
    public static int PICKPOSITION = 3405;
    public Motor frontLeft, frontRight, backLeft, backRight;
    public DcMotor twinTowerMotor, slide;
    public MecanumDrive mecanum;
    // Declare motor and sensor objects
    public SimpleServo extension, claw_turner, claw, angler;
    public SensorRevTOFDistance distanceSensor, colorSensor;
    public static int slideTopPosition = 1800;
    public static int slideClickPosition = 1020;

    public IMU imu;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    public static boolean approximatelyEqual(double a, double b, double tolerance) {
        return Math.abs(a - b) <= tolerance;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        // Initialize motors and sensors from the hardware map
        frontLeft = new Motor(hardwareMap, "frontleft", Motor.GoBILDA.RPM_312);
        frontRight = new Motor(hardwareMap, "frontright", Motor.GoBILDA.RPM_312);
        backLeft = new Motor(hardwareMap, "backleft", Motor.GoBILDA.RPM_312);
        backRight = new Motor(hardwareMap, "backright", Motor.GoBILDA.RPM_312);
        twinTowerMotor = hardwareMap.get(DcMotor.class, "twintower");
        slide = hardwareMap.get(DcMotor.class, "slides");
        distanceSensor = new SensorRevTOFDistance(hardwareMap, "distance");
        colorSensor = new SensorRevTOFDistance(hardwareMap, "color");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        telemetry.addData("Initialization", "Motors and Sensors Initialized");
        extension = new SimpleServo(hardwareMap, "extension", 0, 300, AngleUnit.DEGREES);
        claw_turner = new SimpleServo(hardwareMap, "claw_turner", 0, 300, AngleUnit.DEGREES);
        claw = new SimpleServo(hardwareMap, "claw", 0, 300, AngleUnit.DEGREES);
        angler = new SimpleServo(hardwareMap, "angler", 0, 300, AngleUnit.DEGREES);
        claw.setInverted(false);
        claw_turner.setInverted(true);
        //Claw set positions: 0 is closed and 0.175 is open for perpendicular 0.275 is open for parallel
        //Extension set positions. 0.05 is equal to 0 and 0.65 is folded
        //Claw turner should be driver2 controlled
        twinTowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        twinTowerMotor.setTargetPosition(0);
        twinTowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        twinTowerMotor.setPower(1);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
        angler.turnToAngle(90);
        claw.setPosition(1);
        // Set motors to brake when power is zero
        Motor[] motors = {frontLeft, frontRight, backLeft, backRight};
        for (Motor motor : motors) {
            DcMotor temp = motor.motor;
            motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Motor", "%s set to BRAKE mode", temp.getDeviceName());
        }
        DcMotor[] encoder_run_motors = {twinTowerMotor, slide};
        for (DcMotor motor : encoder_run_motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Motor", "%s set to BRAKE mode", motor.getDeviceName());
        }
        // Set motor directions to ensure correct movement
        frontRight.setInverted(false);
        backRight.setInverted(true);
        frontLeft.setInverted(true);
        backLeft.setInverted(true);
        twinTowerMotor.setDirection(DcMotor.Direction.REVERSE);
        mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        telemetry.addData("Initialization", "Motor Directions Set");
        telemetry.addData("Slide Motor Current Position", slide.getCurrentPosition());
        telemetry.addData("Twin Tower Motor Current Position", twinTowerMotor.getCurrentPosition());
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        // Main loop to handle robot operation during teleop
        while (opModeIsActive()) {
            angler.setPosition(0.025);
            telemetry.addData("Status", "Running");
            handleFieldCentricMovement();
            handleIntake();
            handleOuttake();
            telemetry.update();
            idle();
        }
    }

    private void handleFieldCentricMovement() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // Get current bot heading.
        double botHeading = orientation.getYaw(AngleUnit.RADIANS);
        if (gamepad1.start || (orientation.getYaw(AngleUnit.DEGREES) == 0)|| (orientation.getYaw(AngleUnit.DEGREES) != orientation.getYaw(AngleUnit.DEGREES))) {
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();
        }

        // Set multipliers for drive, strafe, and turn
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

        // Raw input from gamepad
        double turn = gamepad1.right_stick_x * turnMultiplier;
        double strafe = gamepad1.left_stick_x * strafeMultiplier;
        double drive = -gamepad1.left_stick_y * driveMultiplier;


        // Adjust inputs for field-centric control
        double adjustedStrafe = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
        double adjustedDrive = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

        // Calculate motor powers for mecanum wheels
        double denominator = Math.max(Math.abs(adjustedDrive) + Math.abs(adjustedStrafe) + Math.abs(turn), 1);
        double frontLeftPower = (adjustedDrive + adjustedStrafe + turn) / denominator;
        double backLeftPower = (adjustedDrive - adjustedStrafe + turn) / denominator;
        double frontRightPower = (adjustedDrive - adjustedStrafe - turn) / denominator;
        double backRightPower = (adjustedDrive + adjustedStrafe - turn) / denominator;

        // Set motor powers
        frontLeft.set(frontLeftPower);
        backLeft.set(backLeftPower);
        frontRight.set(frontRightPower);
        backRight.set(backRightPower);

        // Add telemetry data
        telemetry.addData("Drive Inputs", "Strafe: %.2f, Drive: %.2f, Turn: %.2f", strafe, drive, turn);
        telemetry.addData("Adjusted Inputs", "Strafe: %.2f, Drive: %.2f", adjustedStrafe, adjustedDrive);
        telemetry.addData("Orientation", "Yaw (heading): %.2f", orientation.getYaw(AngleUnit.DEGREES));
    }

    private void handleIntake() {
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
        if (gamepad2.a) {
            claw.setPosition(0.175);
            telemetry.addData("Claw", "Perpendicular Open Position");
        }
        if (gamepad2.b) {
            claw.setPosition(0.275);
            telemetry.addData("Claw", "Parallel Open Position");
        }
        if (gamepad2.right_trigger > 0) {
            claw_turner.setPosition(claw_turner.getPosition() + (gamepad2.right_trigger * 0.0075));
        }
        if (gamepad2.left_trigger > 0) {
            claw_turner.setPosition(claw_turner.getPosition() - (gamepad2.left_trigger * 0.0075));
        }
        telemetry.addData("Claw Turner Position", claw_turner.getPosition());
        if (gamepad2.x) {
            claw.setPosition(0);
            telemetry.addData("Claw", "Closed Position");
        }

        if (gamepad1.x) {
            twinTowerMotor.setTargetPosition(PICKPOSITION);
            twinTowerMotor.setPower(1);
            extension.setPosition(0.05);
            claw_turner.setPosition(0);
            telemetry.addData("Twin Tower Preset", "Picking Position");
        }
        if (gamepad1.y) {
            extension.setPosition(0.33);
            twinTowerMotor.setTargetPosition(1500);
            twinTowerMotor.setPower(1);
            telemetry.addData("Twin Tower Preset", "Collapsed Position");
        } else {
            twinTowerMotor.setPower(1);
            telemetry.addData("Twin Tower", "Holding Position");
        }

        telemetry.addData("Twin Tower Motor Current Position", twinTowerMotor.getCurrentPosition());
    }


    private void handleOuttake() {
        boolean slideGoingDown = slide.getTargetPosition() > slide.getCurrentPosition();
        boolean slideInRange = slide.getCurrentPosition() >= slideClickPosition-5 && slide.getCurrentPosition() <= slideClickPosition+2;
        if (gamepad1.dpad_up) {
            slide.setTargetPosition(slideTopPosition);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1.0);
            telemetry.addData("Slide", "Moving to Top Chamber Position");

        }
        if (gamepad1.dpad_down) {
            if (slide.getCurrentPosition() > slideClickPosition+10) {
                slide.setTargetPosition(slideClickPosition);
            } else if (slideInRange) {
                slide.setTargetPosition(0);
                claw.setPosition(0.8);
            }
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1.0);
            telemetry.addData("Slide", "Reset to Initial Position");
        }
        if (slideGoingDown && slideInRange) {
            claw.setPosition(0.8);
        }
        if (gamepad1.dpad_right) {
            claw.setPosition(1);
        }
        if (gamepad1.dpad_left) {
            claw.setPosition(0.8);
        }
        if (slide.getCurrentPosition() < 50 && (colorSensor.getDistance(DistanceUnit.CM) >1.5 && colorSensor.getDistance(DistanceUnit.CM) < 1.9)) {
            claw.setPosition(1);
        }
/*
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance Sensor", "Distance: %.2f cm", distance);

        if (distance < lowerdistance) {
            telemetry.addData("Distance", "Too Close. Move Back: %.2f cm", lowerdistance - distance);
        } else if (distance > upperdistance) {
            telemetry.addData("Distance", "Too Far. Move Forward: %.2f cm", distance - upperdistance);
        } else {
            telemetry.addData("Distance", "Correct Distance. Proceed");
        }
*/
    }
}
/*
  Cool things that I should implement:
  TODO: Implement Bulk Reading
  TODO: Make it so that the claw automatically closes when it detects a sample using a distance sensor
  TODO: Switch to Pedro Pathing - IN PROGRESS
  TODO: Switch to finite state/state machine system
  TODO: Port everything to FTC Lib. start using the command and subsystem system
  TODO: place control and expansion hubs in the twin towers
  TODO: Use TOF Distance sensosr(s) to detect when driver has reached correct distance from submersible wall to hang the specimen and same for the perimeter.
  TODO: Do Autonomous
 */