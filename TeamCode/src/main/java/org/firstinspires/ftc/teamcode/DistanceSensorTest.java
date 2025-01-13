package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class DistanceSensorTest extends LinearOpMode{

        @Override
        public void runOpMode() throws InterruptedException {
            waitForStart();
            while (opModeIsActive()) {
                telemetry.addData("Distance", hardwareMap.get(DistanceSensor.class, "distance").getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }
}
