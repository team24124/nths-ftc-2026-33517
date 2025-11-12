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

        // Driving
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.5,
                true
        );

        // Flywheel Control
        if (gamepad1.right_trigger > 0.1) {
            double power = gamepad1.right_trigger;
            leftBigFlywheel.setPower(power);
            rightBigFlywheel.setPower(power);
            rightSmallFlywheel.setPower(power);
            leftSmallFlywheel.setPower(power);
        } else {
            leftBigFlywheel.setPower(0);
            rightBigFlywheel.setPower(0);
            leftSmallFlywheel.setPower(0);
            rightSmallFlywheel.setPower(0);
        }

        // Debug info
        telemetry.addData("Trigger Value", gamepad1.right_trigger);
        telemetry.addData("Left Flywheel Power", leftBigFlywheel.getPower());
        telemetry.addData("Right Flywheel Power", rightBigFlywheel.getPower());
        telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());

        telemetry.update();
    }
}