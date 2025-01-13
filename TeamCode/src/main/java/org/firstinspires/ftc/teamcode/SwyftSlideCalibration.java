package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SwyftSlideCalibration")
public class SwyftSlideCalibration extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor slide;
        slide = hardwareMap.get(DcMotor.class, "slides");

        // Set up the motor with encoder settings
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // Use right and left triggers to manually extend or retract the slide
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            slide.setPower(power);

            // Telemetry for debugging and calibration purposes
            telemetry.addData("Current Position (ticks)", slide.getCurrentPosition());
            telemetry.addData("Instructions", "Extend the slide to the desired maximum position and note the tick value");
            telemetry.update();
        }

        // Stop the motor once the op mode is no longer active
        slide.setPower(1);
    }
}
