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

    // Speed Adjustments
    // Speed multiplier (MAX IS 1)
    double microSpeed = 0.10; // for micro adjustment speed
    double regularSpeed = 0.80; // for regular movement speed
    double turnSpeed = 0.50; // for rotation speed
    double flywheelSpeed = 1300; // for flywheel speed

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize the flywheels
        leftBigFlywheel = hardwareMap.get(DcMotorEx.class, "leftBigFlywheel");
        leftBigFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBigFlywheel = hardwareMap.get(DcMotorEx.class, "rightBigFlywheel");
        rightBigFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        // Movement & Flywheel Adjustments
        double adjustmentIncrement = 0.05;

        // Joystick Movement Variables
        double line = -gamepad1.left_stick_y * regularSpeed;
        double strafe = -gamepad1.left_stick_x * regularSpeed;
        double turn = -gamepad1.right_stick_x * turnSpeed;

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

        // Big Flywheel Control
        if (gamepad1.left_trigger > 0.1) {
            double power = gamepad1.left_trigger;
            rotateFlywheel(power);
        } else {
            rotateFlywheel(0);
        }

        // Small Flywheel Control
        if (gamepad1.right_trigger > 0.1 && gamepad1.left_trigger > 0.1) {
            double power = gamepad1.right_trigger;
            rotateSmallFlywheel(power);
        } else {
            rotateSmallFlywheel(0);
        }

        // Speed Adjustment Controls
        if (gamepad1.xWasPressed()) {
            flywheelSpeed = 1350;
        } else if (gamepad1.bWasPressed()) {
            flywheelSpeed = 1550;
        } else if (gamepad1.yWasPressed()) {
            if (flywheelSpeed <= 2400) {
                flywheelSpeed += 50;
            }
        } else if (gamepad1.aWasPressed()) {
            if (flywheelSpeed >= 100) {
                flywheelSpeed -= 50;
            }
        }

        telemetryUpdate();
    }

    private void telemetryUpdate() {
        // Controls Manual
        telemetry.addLine("====CONTROLS====");
        telemetry.addLine("Left Joystick: Movement");
        telemetry.addLine("Right Joystick: Rotation");
        telemetry.addLine("Right Joystick Button: Rotate 180 degrees clockwise");
        telemetry.addLine("Right Trigger: Small flywheel");
        telemetry.addLine("Left Trigger: Big flywheel");
        telemetry.addLine("D-Pad: Microadjustments for movement");
        telemetry.addLine("Left + Right Bumper: Microadjustments for rotation");
        telemetry.addLine("X: Decrease default movement speed");
        telemetry.addLine("B: Increase default movement speed");
        telemetry.addLine("A: Decrease default flywheel speed");
        telemetry.addLine("Y: Increase default flywheel speed");

        // Info
        telemetry.addLine("\n====ROBOT INFO====");
        telemetry.addData("Current Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Movement Speed", regularSpeed);
        telemetry.addData("Turning Speed", turnSpeed);
        telemetry.addData("Flywheel Targeted Velocity ==", flywheelSpeed);
        telemetry.addData("Flywheel Real-Time Velocity", leftBigFlywheel.getVelocity());

        telemetry.update();
    }

    private void rotateFlywheel(double power) {
        if (power != 0) {
            leftBigFlywheel.setVelocity(flywheelSpeed);
            rightBigFlywheel.setVelocity(flywheelSpeed);
        } else {
            leftBigFlywheel.setVelocity(0);
            rightBigFlywheel.setVelocity(0);
        }
    }

    private void rotateSmallFlywheel(double power) {
        rightSmallFlywheel.setPower(power);
        leftSmallFlywheel.setPower(power);
    }
}