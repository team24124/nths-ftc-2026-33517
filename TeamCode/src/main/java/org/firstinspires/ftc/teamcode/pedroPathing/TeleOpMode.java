package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.teamcode.pedroPathing.SharedPoseStorage.Team.RED;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleOpMode extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;

    private DcMotorEx flywheel, flywheel2, intake;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private CRServo servos;

    /** Constants **/
    double microSpeed = 0.10; // for micro adjustment speed
    double regularSpeed = 0.80; // for regular movement speed
    double flywheelSpeed = 3000.0; // flywheel speed
    double targettedFlywheelSpeed = 1200.0; // speed to target for shooting
    double turnSpeed = 0.50; // for rotation speed
    double slowParkPower = 0.2; // for parking correction speed
    int rumbleTime = 250; // in milliseconds

    // Add these new fields for voltage compensation
    private double voltageMultiplier = 1.0;

    private boolean isRotatingToTarget = false;
    private double targetHeading = 0;
    private boolean rightStickPressed = false;
    private boolean leftStickPressed = false;
    private boolean debounce = false;
    private boolean reachedVelocity = false;
    private boolean autoParkFirstRun = false;

    // Intake control
    private boolean intakeToggle = false;
    private double intakePower = 1.0; // 1.0 = forward & -1.0 = reverse

    // Positioning info
    private SharedPoseStorage.Team selectedTeam = SharedPoseStorage.Team.RED;
    private boolean teamSelected = false;
    private Pose startPose, basePose, scorePose;

    // Driver Assist Toggles
    private boolean autoParking = false;
    private boolean autoScoring = false;

    // Quick Rotation Angle
    double quickRotationAngle = 180.0;

    /** This method configures the starting positions and positioning system **/
    public void setupPosesForTeam() {
        // Set positions based on selected team
       if (selectedTeam == SharedPoseStorage.Team.RED) {
           basePose = new Pose(38.65, 33.25, Math.toRadians(0));
           scorePose = new Pose(72, 132, Math.toRadians(0));
       } else {
           basePose = new Pose(105, 33, Math.toRadians(180));
           scorePose = new Pose(72, 132, Math.toRadians(180));
       }
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize the motors and servos
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        servos = hardwareMap.get(CRServo.class, "servos");

        // Set wheels to brake mode
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Flywheel PIDF tuning
        double p = 0.7; // Fine tune speed
        double i = 0.0; // Fix steady state error/voltage drop
        double d = 2.0; // Dampen oscillations
        double f = 5.0; // Power to reach speed

        flywheel.setVelocityPIDFCoefficients(p, i, d, f);
        flywheel2.setVelocityPIDFCoefficients(p, i, d, f);

        // Set zero power behaviour of the flywheel
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reverse direction
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize the visualizer in panels
        Drawing.init();
    }

    @Override
    public void stop() {
        SharedPoseStorage.poseAvailable = false;
        SharedPoseStorage.teamAvailable = false;
        super.stop();
    }

    @Override
    public void start() {
        setupPosesForTeam();

        // Check if we have a team from autonomous
        if (SharedPoseStorage.teamAvailable) {
            selectedTeam = SharedPoseStorage.currentTeam;
            teamSelected = true;
            setupPosesForTeam();
        } else {
            teamSelected = false;
        }

        // Check if we have a pose from autonomous
        if (SharedPoseStorage.poseAvailable) {
            // Use the pose from autonomous
            follower.setStartingPose(SharedPoseStorage.currentPose);
            telemetry.addLine("Loaded pose from Autonomous!");
            telemetry.addData("X", SharedPoseStorage.currentPose.getX());
            telemetry.addData("Y", SharedPoseStorage.currentPose.getY());
            telemetry.addData("Heading", Math.toDegrees(SharedPoseStorage.currentPose.getHeading()));
        } else {
            teamSelected = false;
        }

        telemetry.update();
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        // Joystick Movement Variables
        double line = -gamepad1.left_stick_y * regularSpeed;
        double strafe = -gamepad1.left_stick_x * regularSpeed;
        double turn = -gamepad1.right_stick_x * turnSpeed;

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

        // Intake toggle
        if (gamepad1.bWasPressed()) {
            if (intake.getPower() == 0.0) {
                intakeToggle = true;
                intake.setPower(intakePower);
            } else {
                intakeToggle = false;
                intake.setPower(0.0);
            }
        }

        // Intake direction toggle
        if (gamepad1.xWasPressed()) {
            intakePower = -intakePower;

            if (intakeToggle) {
                intake.setPower(intakePower);
            }
        }

        // Set gamepad controls
        if (!autoParking && !autoScoring) {
            follower.setTeleOpDrive(line, strafe, turn, true);
        }

        // Big Flywheel Control
        if (gamepad1.left_trigger >= 0.5 && !debounce) {
            debounce = true;
            if (Math.abs(flywheel.getVelocity()) == 0) {
                rotateFlywheel(flywheelSpeed);
            } else {
                rotateFlywheel(0);
            }
        } else if (gamepad1.left_trigger < 0.5) {
            debounce = false;
        }

        // Check if up to speed
        if (Math.abs(flywheel.getVelocity()) >= targettedFlywheelSpeed && !reachedVelocity) {
            gamepad1.rumble(rumbleTime); // Let driver know flywheel is up to speed
            reachedVelocity = true;
        } else if (Math.abs(flywheel.getVelocity()) < targettedFlywheelSpeed - 25 && reachedVelocity) {
            reachedVelocity = false;
        }

        // Small Flywheel Control
        if (gamepad1.right_trigger >= 0.1 && Math.abs(flywheel.getVelocity()) >= 0) {
            if (!intakeToggle) {
                intake.setPower(intakePower);
            }

            if (Math.abs(flywheel.getVelocity()) > targettedFlywheelSpeed / 2) {
                servos.setPower(1.0);
            }
        } else {
            if (!intakeToggle) {
                intake.setPower(0.0);
            }

            servos.setPower(0.0);
        }

        // Auto Score with toggle
        if (gamepad1.yWasPressed() && teamSelected) {
            if (!autoScoring && !autoParking) {
                PathChain toScore = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), scorePose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                        .build();
                follower.followPath(toScore, true);
                autoScoring = true;
            } else { // Stop AutoScore if driver hits A while AutoScore is happening
                resetStates();
            }
        }

        // Auto Park with toggle
        if (gamepad1.aWasPressed() && teamSelected) {
            if (!autoScoring && !autoParking) {
                PathChain toBase = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), basePose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), basePose.getHeading())
                        .build();
                follower.followPath(toBase, true);
                autoParking = true;
                autoParkFirstRun = true;
            } else { // Stop AutoPark if driver hits Y while an AutoPark is happening
                resetStates();
            }
        }

        if (autoParking || autoScoring) {
            // Kill Switch to cancel driver assist if driver makes any joystick moves
            if (Math.abs(gamepad1.left_stick_y) >= 0.1 || Math.abs(gamepad1.left_stick_x) >= 0.1 || Math.abs(gamepad1.right_stick_x) >= 0.1) {
                resetStates();
            }

            // Check if auto pathing has finished
            if (!follower.isBusy()) {
                if (autoParking && autoParkFirstRun) {
                    // First run completed, now do it again slowly
                    PathChain toBase = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), basePose))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), basePose.getHeading())
                            .build();
                    follower.setMaxPower(slowParkPower);
                    follower.followPath(toBase, true);
                    autoParkFirstRun = false;
                } else {
                    // Second run completed (or auto score finished)
                    resetStates();
                }
            }
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

        telemetryUpdate();
    }

    /** This method resets the state of any autonomous teleop features **/
    private void resetStates() {
        follower.setMaxPower(1.0);
        follower.breakFollowing();
        autoParking = false;
        autoScoring = false;
        follower.startTeleopDrive();
    }

    /** This method updates the telemetry information on the driver hub/panels **/
    private void telemetryUpdate() {
        // Info
        telemetry.addLine("====ROBOT INFO====");
        telemetry.addData("Movement Speed", regularSpeed);
        telemetry.addData("Turning Speed", turnSpeed);
        telemetry.addData("Flywheel Targeted Velocity", targettedFlywheelSpeed);
        telemetry.addData("Flywheel Real-Time Velocity", Math.abs(flywheel.getVelocity()));
        telemetry.addData("Intake Status", (intake.getPower()) == 0 ? "Off" : "On");
        telemetry.addData("Intake Direction", (intakePower >= 0.0 ? "Forward" : "Reversed"));

        if (teamSelected) {
            telemetry.addLine("\n====DRIVER ASSIST & POSITIONING SYSTEM====");
            telemetry.addData("Current Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("AutoPark Status", (autoParking ? "Parking.." : "Idle"));
            telemetry.addData("AutoScore Status", (autoScoring ? "Scoring.." : "Idle"));
        } else {
            telemetry.addData("Driver Assist & Positioning System", "UNAVAILABLE");
        }

        // Controls Manual
        telemetry.addLine("\n====CONTROLS====");
        telemetry.addLine("Left Joystick: Movement");
        telemetry.addLine("Right Joystick: Rotation");
        telemetry.addLine("Right Joystick Button: Rotate 180 degrees clockwise");
        telemetry.addLine("Right Trigger (Hold): Small flywheel");
        telemetry.addLine("Left Trigger (Click): Big flywheel Toggle");
        telemetry.addLine("D-Pad: Microadjustments for movement");
        telemetry.addLine("Left + Right Bumper: Microadjustments for rotation");
        telemetry.addLine("Left Action Button: Flip Intake Direction");
        telemetry.addLine("Right Action Button: Toggle Intake");

        if (teamSelected) {
            telemetry.addLine("Bottom Action Button: Auto Park");
            telemetry.addLine("Top Action Button: Auto Align");
            Drawing.drawDebug(follower);
        }

        telemetry.update();
    }

    private void rotateFlywheel(double speed) {
        flywheel.setVelocity(speed);
        flywheel2.setVelocity(speed);
    }
}