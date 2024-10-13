package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TwinTowerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize the motors for the twin towers
        DcMotor twinTowerMotor = hardwareMap.get(DcMotor.class, "twintower");
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.right_trigger>0){
                twinTowerMotor.setPower(gamepad1.right_trigger/1.5);
            }
            if (gamepad1.left_trigger>0){
                twinTowerMotor.setPower(-gamepad1.left_trigger/2);
            }
        }
    }
}
