package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleOpMode extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;

    private DcMotorEx leftBigFlywheel, rightBigFlywheel;
    private CRServo leftSmallFlywheel, rightSmallFlywheel;

    private boolean isRotatingToTarget = false;
    private double targetHeading = 0;
    private boolean rightStickPressed = false;
    private boolean leftStickPressed = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize the flywheels
        leftBigFlywheel = hardwareMap.get(DcMotorEx.class, "leftBigFlywheel");
        rightBigFlywheel = hardwareMap.get(DcMotorEx.class, "rightBigFlywheel");
        leftSmallFlywheel = hardwareMap.get(CRServo.class, "leftSmallFlywheel");
        rightSmallFlywheel = hardwareMap.get(CRServo.class, "rightSmallFlywheel");

        // Reverse motor directions as needed
        leftBigFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSmallFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power behaviour of the flywheels
        leftBigFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBigFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        // Joystick Movement Variables
        double line = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x * 0.5;

        // Speed of Micro Adjustments
        double microSpeed = 0.1; // Adjust this (lower = slower)

        // Quick Rotation Angle
        double quickRotationAngle = 180.0;

        // Micro Movement Control
        if (gamepad1.dpad_up) {
            line = microSpeed;
            strafe = 0.0;
        } else if (gamepad1.dpad_down) {
            line = -microSpeed;
            strafe = 0.0;
        } else if (gamepad1.dpad_right) {
            line = 0.0;
            strafe = -microSpeed;
        } else if (gamepad1.dpad_left) {
            line = 0.0;
            strafe = microSpeed;
        }

        // Micro Rotation Control
        if (gamepad1.right_bumper) {
            turn = -microSpeed;
        } else if (gamepad1.left_bumper) {
            turn = microSpeed;
        }

        // Quick Rotation Control
        if (gamepad1.right_stick_button && !rightStickPressed && !isRotatingToTarget) {
            rightStickPressed = true;
            double currentHeading = Math.toDegrees(follower.getPose().getHeading());
            targetHeading = Math.toRadians(currentHeading - quickRotationAngle);
            isRotatingToTarget = true;
        } else if (!gamepad1.right_stick_button) {
            rightStickPressed = false;
        }

        // If rotating to target, override turn control
        if (isRotatingToTarget) {
            double currentHeading = follower.getPose().getHeading();
            double headingError = targetHeading - currentHeading;

            // Normalize error to -PI to PI
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            // Stop if close enough (within 1 degree)
            if (Math.abs(Math.toDegrees(headingError)) < 1.0) {
                turn = 0;
                isRotatingToTarget = false;
            } else {
                // Proportional control - turn towards target
                turn = headingError * 0.5; // Adjust multiplier for speed
            }
        }

        // Set gamepad controls
        follower.setTeleOpDrive(line, strafe, turn, true);

        // Flywheel Control
        if (gamepad1.right_trigger > 0.1) {
            double power = gamepad1.right_trigger;
            rotateFlywheel(power);
        } else {
            rotateFlywheel(0);
        }

        telemetryUpdate();
    }

    private void telemetryUpdate() {
        // Controls Manual
        telemetry.addLine("====CONTROLS====");
        telemetry.addLine("Left Joystick: Movement");
        telemetry.addLine("Right Joystick: Rotation");
        telemetry.addLine("Right Joystick Button: Rotate 180 degrees clockwise");
        telemetry.addLine("Right Trigger: Flywheel");
        telemetry.addLine("D-Pad: Microadjustments for movement");
        telemetry.addLine("Left + Right Bumper: Microadjustments for rotation");

        // Info
        telemetry.addLine("\n====ROBOT INFO====");
        telemetry.addData("Current Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }

    private void rotateFlywheel(double power) {
        leftBigFlywheel.setPower(power);
        rightBigFlywheel.setPower(power);
        rightSmallFlywheel.setPower(power);
        leftSmallFlywheel.setPower(power);
    }
}