package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class OuttakeCalibration extends LinearOpMode {

    // Configuration parameters
    public static int MAX_EXTENSION_TICKS = 99999; // Max extension for slide motor
    public static double CLAW_POSITION = 0;
    public static double SWYFT_SLIDE_MULTIPLIER = 1993.6; // Multiplier for slide motor calculation
    public static double ANGLER_POSITION = 0;

    @Override
    public void runOpMode() {
        // Hardware initialization
        DcMotor slide = hardwareMap.get(DcMotor.class, "slides");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        DcMotorEx twinTowerMotor = hardwareMap.get(DcMotorEx.class, "twintower");
        DistanceSensor color = hardwareMap.get(DistanceSensor.class, "color");
        Servo angler = hardwareMap.get(Servo.class, "angler");

        // Slide motor setup
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Twin tower motor setup
        twinTowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        twinTowerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Slide control
            telemetry.addData("Distance", hardwareMap.get(DistanceSensor.class, "distance").getDistance(DistanceUnit.CM));
            double rtpower = gamepad1.right_trigger * SWYFT_SLIDE_MULTIPLIER;
            double ltpower = gamepad1.left_trigger * SWYFT_SLIDE_MULTIPLIER;

            if (gamepad1.right_trigger > 0) {
                int targetPosition = slide.getCurrentPosition() + (int) rtpower;
                targetPosition = Math.min(targetPosition, MAX_EXTENSION_TICKS);
                slide.setTargetPosition(targetPosition);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1.0);
            } else if (gamepad1.left_trigger > 0) {
                int targetPosition = slide.getCurrentPosition() - (int) ltpower;
                targetPosition = Math.max(targetPosition, 0);
                slide.setTargetPosition(targetPosition);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1.0);
            } else {
                slide.setPower(1);
            }

            // Intake/Outtake calibration
            if (gamepad1.right_stick_button || gamepad1.left_stick_button) {
                //correct angler zero is 0.075
                //correct claw close is 1
                //correct claw open is 0.8
                claw.setPosition(CLAW_POSITION);
                angler.setPosition(ANGLER_POSITION);
            }

            if (gamepad1.y) {
                if (gamepad1.right_trigger > 0) {
                    int targetPosition = twinTowerMotor.getCurrentPosition() + (int) (gamepad1.right_trigger * SWYFT_SLIDE_MULTIPLIER);
                    twinTowerMotor.setTargetPosition(targetPosition);
                    twinTowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    twinTowerMotor.setPower(1.0);
                } else if (gamepad1.left_trigger > 0) {
                    int targetPosition = twinTowerMotor.getCurrentPosition() - (int) (gamepad1.left_trigger * SWYFT_SLIDE_MULTIPLIER);
                    twinTowerMotor.setTargetPosition(targetPosition);
                    twinTowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    twinTowerMotor.setPower(1.0);
                }
            }

            // Telemetry for debugging
            telemetry.addData("Slide Current Position", slide.getCurrentPosition());
            telemetry.addData("Slide Target Position", slide.getTargetPosition());
            telemetry.addData("Twin Tower Motor Position", twinTowerMotor.getCurrentPosition());
            telemetry.addData("Twin Tower Motor Power", twinTowerMotor.getPower());
            telemetry.addData("Distance", color.getDistance(DistanceUnit.CM));
            telemetry.update();

            idle();
        }
    }
}
