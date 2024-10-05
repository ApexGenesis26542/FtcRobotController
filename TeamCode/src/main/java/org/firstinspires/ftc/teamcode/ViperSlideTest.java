package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ViperSlideTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftViper = hardwareMap.get(DcMotor.class, "leftviper");
        DcMotor rightViper = hardwareMap.get(DcMotor.class, "rightviper");
        //leftViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //rightViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        waitForStart();
        while (opModeIsActive()) {
            while (gamepad1.dpad_up) {
                leftViper.setDirection(DcMotorSimple.Direction.FORWARD);
                rightViper.setDirection(DcMotorSimple.Direction.FORWARD);
                leftViper.setPower(gamepad1.right_trigger);
                rightViper.setPower(gamepad1.right_trigger);
            }
            while (gamepad1.dpad_down) {
                leftViper.setDirection(DcMotorSimple.Direction.REVERSE);
                rightViper.setDirection(DcMotorSimple.Direction.REVERSE);
                leftViper.setPower(gamepad1.right_trigger);
                rightViper.setPower(gamepad1.right_trigger);
            }
        }
    }
}