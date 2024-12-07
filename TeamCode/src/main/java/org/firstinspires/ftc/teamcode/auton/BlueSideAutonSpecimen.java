package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class BlueSideAutonSpecimen extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-12.00, 62.50, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TwinTower twinTower = new TwinTower(hardwareMap);

        Action trajectoryAction = drive.actionBuilder(initialPose).splineToLinearHeading(new Pose2d(0.00, 53.00, Math.toRadians(270.00)), Math.toRadians(270.00)).stopAndAdd(twinTower.hangSpecimen()).splineToLinearHeading(new Pose2d(0.00, 32.50, Math.toRadians(270.00)), Math.toRadians(270.00)).splineToLinearHeading(new Pose2d(0.00, 47.00, Math.toRadians(270.00)), Math.toRadians(270.00)).splineTo(new Vector2d(-21, 54.00), Math.toRadians(160)).splineTo(new Vector2d(-53.00, 60.00), Math.toRadians(180.00)).stopAndAdd(twinTower.resetPosition()).build();

        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(trajectoryAction);
        }
    }

    public class TwinTower {
        private DcMotorEx twinTowerMotor;

        public TwinTower(HardwareMap hardwareMap) {
            twinTowerMotor = hardwareMap.get(DcMotorEx.class, "twintower");
            twinTowerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            twinTowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            twinTowerMotor.setTargetPosition(0);
            twinTowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public Action hangSpecimen() {
            return new HighChamberHang();
        }

        public Action resetPosition() {
            return new ResetPosition();
        }

        public class HighChamberHang implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    twinTowerMotor.setTargetPosition(-2100);
                    twinTowerMotor.setPower(1);
                    initialized = true;
                }
                int currentPosition = twinTowerMotor.getCurrentPosition();
                packet.put("TwinTower Position", currentPosition);
                return twinTowerMotor.isBusy();
            }
        }

        // New action to reset the TwinTower motor to position 0
        public class ResetPosition implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    twinTowerMotor.setTargetPosition(0);
                    twinTowerMotor.setPower(1);
                    initialized = true;
                }
                int currentPosition = twinTowerMotor.getCurrentPosition();
                packet.put("TwinTower Position", currentPosition);
                return twinTowerMotor.isBusy();
            }
        }
    }
}
