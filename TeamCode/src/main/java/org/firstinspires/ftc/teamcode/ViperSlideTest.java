package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ViperSlideTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize the motors for the viper slides
        DcMotor leftViper = hardwareMap.get(DcMotor.class, "leftviper");
        DcMotor rightViper = hardwareMap.get(DcMotor.class, "rightviper");

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                // Move upwards with reduced power for controlled movement
                // Using a lower power value to ensure smooth and controlled ascent
                leftViper.setPower(0.3);
                rightViper.setPower(0.3);
            } else if (gamepad1.dpad_down) {
                // Move downwards with full power
                // Negative power allows the slide to move downwards
                leftViper.setPower(-0.5);
                rightViper.setPower(-0.5);
            } else {
                // Apply a small power to hold position and prevent sagging
                // This helps counteract gravity and maintain the current position
                leftViper.setPower(0.05);
                rightViper.setPower(0.05);
            }

            // Display telemetry data for debugging purposes
            // Showing the current power applied to each motor to help with diagnostics
            telemetry.addData("Left Viper Power", leftViper.getPower());
            telemetry.addData("Right Viper Power", rightViper.getPower());
            telemetry.update();
        }
    }
}