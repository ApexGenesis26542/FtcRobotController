package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IntakeCalibration extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo rotator = hardwareMap.get(Servo.class, "rotator");
        Servo angler = hardwareMap.get(Servo.class, "angler");
        DcMotorEx twinTowerMotor = hardwareMap.get(DcMotorEx.class, "twintower");
        angler.setDirection(Servo.Direction.REVERSE);

        // Configure the motor to use the encoder for more precise control
        twinTowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        twinTowerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                if (gamepad1.dpad_right) {
                    angler.setPosition(0);
                }
                if (gamepad1.dpad_up) {
                    angler.setDirection(Servo.Direction.REVERSE);
                    if (angler.getPosition() < 1) {
                        angler.setPosition(angler.getPosition() + 0.003);
                        sleep(100);
                    }
                }
                if (gamepad1.dpad_down) {
                    angler.setDirection(Servo.Direction.FORWARD);
                    if (angler.getPosition() > 0) {
                        angler.setPosition(angler.getPosition() + 0.003);
                        sleep(100);
                    }
                }
            }
            if (gamepad1.b) {
                if (gamepad1.dpad_right) {
                    rotator.setPosition(0);
                }
                if (gamepad1.dpad_up) {
                    rotator.setDirection(Servo.Direction.FORWARD);
                    if (rotator.getPosition() < 1) {
                        rotator.setPosition(rotator.getPosition() + 0.003);
                        sleep(100);
                    }
                }
                if (gamepad1.dpad_down) {
                    rotator.setDirection(Servo.Direction.REVERSE);
                    if (rotator.getPosition() > 0) {
                        rotator.setPosition(rotator.getPosition() + 0.003);
                        sleep(100);
                    }
                }
            }
            if (gamepad1.x) {
                rotator.setPosition(0);
                angler.setPosition(0);
            }
            if (gamepad1.y) {
                if (gamepad1.right_trigger > 0) {
                    int targetPosition = twinTowerMotor.getCurrentPosition() + (int) (gamepad1.right_trigger * 1993.6); // Use encoder PPR value to calculate target
                    twinTowerMotor.setTargetPosition(targetPosition);
                    twinTowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    twinTowerMotor.setPower(1.0);
                }
                if (gamepad1.left_trigger > 0) {
                    int targetPosition = twinTowerMotor.getCurrentPosition() - (int) (gamepad1.left_trigger * 1993.6); // Use encoder PPR value to calculate target
                    twinTowerMotor.setTargetPosition(targetPosition);
                    twinTowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    twinTowerMotor.setPower(1.0);
                }
            }
            telemetry.addData("Current Rotator Angle in Degrees", rotator.getPosition() * 300);
            telemetry.addData("Current Rotator Position", rotator.getPosition());
            telemetry.addData("Current Angler Angle in Degrees", angler.getPosition() * 300);
            telemetry.addData("Current Angler Position", angler.getPosition());
            telemetry.addData("Current Twin Tower Motor Power", twinTowerMotor.getPower());
            telemetry.addData("Current Twin Tower Motor Position", twinTowerMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
