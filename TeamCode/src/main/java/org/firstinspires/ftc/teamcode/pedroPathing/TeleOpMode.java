package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleOpMode extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;

    private DcMotorEx flywheel;
    private CRServo leftServo, rightServo;

    private boolean isRotatingToTarget = false;
    private double targetHeading = 0;
    private boolean rightStickPressed = false;
    private boolean leftStickPressed = false;
    private boolean debounce = false;

    // Positioning info
    private enum Team {RED, BLUE};
    private Team selectedTeam = Team.RED;
    private boolean teamSelected = false;
    private Pose startPose, basePose, scorePose;

    /*
     * 0: Front of blue goal
     * 1: Front of red goal
     * 2: Left of small launch area
     * 3: Right of small launch area
     */
    private int startPosition = 0;

    // Driver Assist Toggles
    private boolean autoParking = false;
    private boolean autoScoring = false;

    // Speed Adjustments
    // Speed multiplier (MAX IS 1)
    double microSpeed = 0.10; // for micro adjustment speed
    double regularSpeed = 0.80; // for regular movement speed
    double turnSpeed = 0.50; // for rotation speed
    double flywheelSpeed = 200; // for flywheel speed

    // Quick Rotation Angle
    double quickRotationAngle = 180.0;

    /** This method configures the starting positions and positioning system **/
    public void setupPosesForTeam() {
        // Set positions based on selected team
       if (selectedTeam == Team.RED) {
           basePose = new Pose(105.25, 33.25, Math.toRadians(0));
           scorePose = new Pose(84, 12, Math.toRadians(68));
       } else {
           basePose = new Pose(38.65, 33.25, Math.toRadians(180));
           scorePose = new Pose(60, 12, Math.toRadians(112));
       }

        // Set starting positions based
        switch (startPosition) {
            case 0:
                startPose = new Pose(24, 125, Math.toRadians(323));
                break;
            case 1:
                startPose = new Pose(120, 125, Math.toRadians(217));
                break;
            case 2:
                startPose = new Pose(56, 8, Math.toRadians(90));
                break;
            default:
                startPose = new Pose(88, 8, Math.toRadians(90));
                break;
        }
    }

    @Override
    public void init_loop() {
        telemetry.addLine("====AUTOPARK CONFIGURATION====");
        telemetry.addLine("! To enable autopark and position tracking, you need to select a start position !");
        telemetry.addLine("! This feature is optional, however, you will not be able to use autopark or view your position/heading!");
        telemetry.addLine();
        telemetry.addLine("X: Front of the Blue Goal");
        telemetry.addLine("Y: Front of the Red Goal");
        telemetry.addLine("A: Left of the Small Launch Area");
        telemetry.addLine("B: Right of the Small Launch Area");
        telemetry.addLine();

        // Set status message based on position selection
        if (teamSelected) { // Team and starting position was selected
            switch (startPosition) {
                case 0:
                    telemetry.addLine("STATUS: Starting at the front of the BLUE goal");
                    break;
                case 1:
                    telemetry.addLine("STATUS: Starting at the front of the RED goal");
                    break;
                case 2:
                    telemetry.addLine("STATUS: Starting at the BLUE small launch area");
                    break;
                default:
                    telemetry.addLine("STATUS: Starting at the RED small launch area");
                    break;
            }
        } else { // Team and starting position was not selected
            telemetry.addLine("STATUS: Waiting..");
        }

        // Controls to select team
        if (!teamSelected) {
            if (gamepad1.x) {
                selectedTeam = Team.BLUE;
                startPosition = 0;
                teamSelected = true;
            } else if (gamepad1.y) {
                selectedTeam = Team.RED;
                startPosition = 1;
                teamSelected = true;
            }  else if (gamepad1.a) {
                selectedTeam = Team.BLUE;
                startPosition = 2;
                teamSelected = true;
            } else if (gamepad1.b) {
                selectedTeam = Team.RED;
                startPosition = 3;
                teamSelected = true;
            }
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize the flywheel and servo
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");

        // Flywheel PIDF tuning
        flywheel.setVelocityPIDFCoefficients(2, 0, 0, 12.55);

        // Set zero power behaviour of the flywheel
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize the visualizer in panels
        Drawing.init();
    }

    @Override
    public void start() {
        setupPosesForTeam();
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
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

        // Auto Score with toggle
        if (gamepad1.aWasPressed() && teamSelected) {
            if (!autoScoring && !autoParking) {
                Path toScore = new Path(new BezierLine(follower.getPose(), scorePose));
                follower.followPath(toScore, true);
                autoScoring = true;
            } else { // Stop AutoScore if driver hits A while AutoScore is happening
                follower.breakFollowing();
            }
        }

        // Auto Park with toggle
        if (gamepad1.yWasPressed() && teamSelected) {
            if (!autoScoring && !autoParking) {
                Path toBase = new Path(new BezierLine(follower.getPose(), basePose));
                follower.followPath(toBase, true);
                autoParking = true;
            } else { // Stop AutoPark if driver hits Y while an AutoPark is happening
                follower.breakFollowing();
            }
        }

        if (autoParking || autoScoring) {
            // Killswitch to cancel driver assist if driver makes any manual moves
            if (Math.abs(gamepad1.left_stick_y) >= 0.1 || Math.abs(gamepad1.left_stick_x) >= 0.1 || Math.abs(gamepad1.right_stick_x) >= 0.1) {
                follower.breakFollowing();
                autoParking = false;
                autoScoring = false;
            }

            // Check if auto parking has finished
            if (!follower.isBusy()) {
                autoScoring = false;
                autoParking = false;
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

        // Set gamepad controls
        if (!autoParking && !autoScoring) {
            follower.setTeleOpDrive(line, strafe, turn, true);
        }

        // Big Flywheel Control
        if (gamepad1.left_trigger >= 0.5 && !debounce) {
            debounce = true;
            if (flywheel.getVelocity() == 0) {
                rotateFlywheel(flywheelSpeed);
            } else {
                rotateFlywheel(0);
            }
        } else if (gamepad1.left_trigger < 0.5) {
            debounce = false;
        }

        // Small Flywheel Control
        if (gamepad1.right_trigger >= 0.1 &&  flywheel.getVelocity() >= 0) {
            rotateServos(1.0);
        } else {
            rotateServos(0.0);
        }

        telemetryUpdate();
    }

    /** This method updates the telemetry information on the driver hub/panels **/
    private void telemetryUpdate() {
        // Info
        telemetry.addLine("====ROBOT INFO====");
        telemetry.addData("Movement Speed", regularSpeed);
        telemetry.addData("Turning Speed", turnSpeed);
        telemetry.addData("Flywheel Targeted Velocity", flywheelSpeed);
        telemetry.addData("Flywheel Real-Time Velocity", flywheel.getVelocity());

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

        if (teamSelected) {
            telemetry.addLine("Y: AutoPark");
            telemetry.addLine("A: AutoScore");
            Drawing.drawDebug(follower);
        }

        telemetry.update();
    }

    private void rotateFlywheel(double speed) {
        flywheel.setVelocity(speed);
    }

    private void rotateServos(double power) {
        leftServo.setPower(power);
        rightServo.setPower(power);
    }
}