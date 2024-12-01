package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class SwyftSlideTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor slide;
        slide = hardwareMap.get(DcMotor.class, "slides");

        // Set up the motor with encoder settings
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor configuration settings
        final int MAX_EXTENSION_TICKS = 810; // Assuming 5377 ticks equals 23 inches based on encoder resolution
        double twinTowerMotorMultiplier = 1993.6; // Example multiplier for target position calculation

        waitForStart();

        while (opModeIsActive()) {
            double rtpower = gamepad1.right_trigger * twinTowerMotorMultiplier;
            double ltpower = gamepad1.left_trigger * twinTowerMotorMultiplier;

            // Move the slide up when right trigger is pressed
            if (gamepad1.right_trigger > 0) {
                int targetPosition = slide.getCurrentPosition() + (int) (gamepad1.right_trigger * twinTowerMotorMultiplier);
                targetPosition = Math.min(targetPosition, MAX_EXTENSION_TICKS); // Soft limit for maximum extension
                slide.setTargetPosition(targetPosition);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(rtpower);
            }
            // Move the slide down when left trigger is pressed
            else if (gamepad1.left_trigger > 0) {
                int targetPosition = slide.getCurrentPosition() - (int) (gamepad1.left_trigger * twinTowerMotorMultiplier);
                targetPosition = Math.max(targetPosition, 0); // Soft limit to prevent negative position
                slide.setTargetPosition(targetPosition);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(ltpower);
            } else {
                // Set motor power to zero if neither trigger is pressed
                slide.setPower(0);
            }

            // Telemetry for debugging purposes
            telemetry.addData("Current Position", slide.getCurrentPosition());
            telemetry.addData("Target Position", slide.getTargetPosition());
            telemetry.update();
        }
    }
}
