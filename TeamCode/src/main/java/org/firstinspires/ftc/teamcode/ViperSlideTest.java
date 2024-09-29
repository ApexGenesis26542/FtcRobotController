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
        while (opModeIsActive()) {
            while (gamepad1.dpad_up) {
                leftViper.setPower(gamepad1.right_trigger);
                rightViper.setPower(gamepad1.right_trigger);
            }
            
        }
    }
}