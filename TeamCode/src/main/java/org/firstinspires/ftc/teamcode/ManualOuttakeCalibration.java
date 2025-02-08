package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class ManualOuttakeCalibration extends LinearOpMode {
    public static int MAX_EXTENSION_TICKS = 2700; // Max extension for slide motor
    public static int slideTopPosition = 1800;
    public static int slideClickPosition = 1020;
    public static int slideBottomPosition = 0;
    public static double SWYFT_SLIDE_MULTIPLIER = 1993.6; // Multiplier for slide motor calculation

    @Override


    public void runOpMode() {
        DcMotor slide = hardwareMap.get(DcMotor.class, "slides");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo angler = hardwareMap.get(Servo.class, "angler");
        DistanceSensor color = hardwareMap.get(DistanceSensor.class, "color");
        // Slide motor setup
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angler.setPosition((double) 90 /300);
        waitForStart();

        while (opModeIsActive()) {
            angler.setPosition(0);
            boolean slideGoingDown = slide.getTargetPosition() > slide.getCurrentPosition();
            boolean slideInRange = slide.getCurrentPosition() >= slideClickPosition-5 && slide.getCurrentPosition() <= slideClickPosition+2;
            double rtpower = gamepad1.right_trigger * SWYFT_SLIDE_MULTIPLIER;
            double ltpower = gamepad1.left_trigger * SWYFT_SLIDE_MULTIPLIER;
            if (gamepad1.dpad_up) {
                slide.setTargetPosition(slideTopPosition);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1.0);
            } else if (gamepad1.dpad_down) {
                if (slide.getCurrentPosition() > slideClickPosition+10) {
                    slide.setTargetPosition(slideClickPosition);
                } else if (slideInRange) {
                    slide.setTargetPosition(slideBottomPosition);
                }
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1.0);
            }
            if (gamepad1.right_stick_button) {
                slide.setPower(1);
            }
            if (gamepad1.left_stick_button) {
                slide.setPower(0);
            }
            if (gamepad1.a) {
                claw.setPosition(1);
            }
            if (gamepad1.b) {
                claw.setPosition(0.8);
            }
            if (slideGoingDown && slideInRange) {
                claw.setPosition(0.8);
            }
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
            }
            telemetry.addData("Slide Position", slide.getCurrentPosition());
            telemetry.addData("Slide Target Position", slide.getTargetPosition());
            telemetry.addData("Distance", color.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

}
