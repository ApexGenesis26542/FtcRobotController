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
public class RedSideAutonSpecimen extends LinearOpMode {

    public class TwinTower {
        private DcMotorEx twinTowerMotor;

        public TwinTower(HardwareMap hardwareMap) {
            twinTowerMotor = hardwareMap.get(DcMotorEx.class, "twintower");
            twinTowerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            twinTowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            twinTowerMotor.setTargetPosition(0);
            twinTowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

        public Action hangSpecimen() {
            return new HighChamberHang();
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

        public Action resetPosition() {
            return new ResetPosition();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(12.50, -62.50, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TwinTower twinTower = new TwinTower(hardwareMap);

        Action trajectoryAction = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(0.00, -53.00, Math.toRadians(90.00)), Math.toRadians(90.00))
                .stopAndAdd(twinTower.hangSpecimen()) // Executes hangSpecimen() after the first segment
                .splineToLinearHeading(new Pose2d(0.00, -32.50, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineToLinearHeading(new Pose2d(0.00, -47.09, Math.toRadians(90.00)), Math.toRadians(90.00))
                .splineTo(new Vector2d(13.94, -46.63), Math.toRadians(-30.05))
                .splineTo(new Vector2d(53.00, -60.00), Math.toRadians(0.00))
                .stopAndAdd(twinTower.resetPosition()) // Resets TwinTower to position 0 after hangSpecimen()
                .build();

        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(trajectoryAction);
        }
    }
}
