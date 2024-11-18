package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IntakeCalibration extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo rotator = hardwareMap.get(Servo.class, "rotator");
        Servo angler = hardwareMap.get(Servo.class, "angler");
        angler.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                if (gamepad1.dpad_right) {
                    rotator.setPosition(0);
                }
                if (gamepad1.dpad_up) {
                    if (rotator.getPosition() < 1) {
                        rotator.setPosition(rotator.getPosition() + 0.003);
                    }
                    if (gamepad1.dpad_down) {
                        rotator.setPosition(rotator.getPosition() - 0.003);
                    }
                }
            }
            if (gamepad1.b) {
                if (gamepad1.dpad_right) {
                    angler.setPosition(0);
                }
                if (gamepad1.dpad_up) {
                    if (rotator.getPosition() < 1) {
                        rotator.setPosition(rotator.getPosition() + 0.003);
                    }
                    if (gamepad1.dpad_down) {
                        rotator.setPosition(rotator.getPosition() - 0.003);
                    }
                }
            }
            telemetry.addData("Current Rotator Angle in Degrees", rotator.getPosition() * 300);
            telemetry.addData("Current Rotator Po   sition", rotator.getPosition());
            telemetry.addData("Current Angler Angle in Degrees", angler.getPosition() * 300);
            telemetry.addData("Current Angler Position", angler.getPosition());
            telemetry.update();
        }
    }
}
