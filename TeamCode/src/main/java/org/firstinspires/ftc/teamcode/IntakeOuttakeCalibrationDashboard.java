package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class IntakeOuttakeCalibrationDashboard extends LinearOpMode {
    public static double anglerpos = 0;
    public static Servo.Direction anglerdirection = Servo.Direction.FORWARD;
    @Override
    public void runOpMode() {
        Servo angler = hardwareMap.get(Servo.class, "angler");
        DcMotorEx twinTowerMotor = hardwareMap.get(DcMotorEx.class, "twintower");
        // Configure the motor to use the encoder for more precise control
        twinTowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        twinTowerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_stick_button || gamepad1.left_stick_button) {
                angler.setDirection(anglerdirection);
                angler.setPosition(anglerpos);
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
            telemetry.addData("Current Angler Angle in Degrees", angler.getPosition() * 300);
            telemetry.addData("Current Angler Position", angler.getPosition());
            telemetry.addData("Current Twin Tower Motor Power", twinTowerMotor.getPower());
            telemetry.addData("Current Twin Tower Motor Position", twinTowerMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
