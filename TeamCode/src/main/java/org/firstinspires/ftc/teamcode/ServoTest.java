package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo testServo = hardwareMap.get(Servo.class, "testservo");
        testServo.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                testServo.setDirection(Servo.Direction.FORWARD);
                testServo.setPosition(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger > 0) {
                testServo.setDirection(Servo.Direction.REVERSE);
                testServo.setPosition(gamepad1.left_trigger);
            } else {
                testServo.setPosition(0);
            }
        }
    }
}