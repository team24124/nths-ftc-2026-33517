package org.firstinspires.ftc.teamcode.pedroPathing;
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
    private CRServo leftServo, rightServo;

    /** Constants **/
    double microSpeed = 0.10; // for micro adjustment speed
    double regularSpeed = 0.80; // for regular movement speed
    double turnSpeed = 0.50; // for rotation speed
    double flywheelSpeed = 1650; // for flywheel speed
    int rumbleTime = 250; // in milliseconds

    private boolean isRotatingToTarget = false;
    private double targetHeading = 0;
    private boolean rightStickPressed = false;
    private boolean leftStickPressed = false;
    private boolean debounce = false;
    private boolean reachedVelocity = false;

    // Intake control
    private boolean intakeToggle = false;
    private double intakePower = 1.0; // 1.0 = forward & -1.0 = reverse

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

    // Quick Rotation Angle
    double quickRotationAngle = 180.0;

    /** This method configures the starting positions and positioning system **/
    public void setupPosesForTeam() {
        // Set positions based on selected team
       if (selectedTeam == Team.RED) {
           basePose = new Pose(38.65, 33.25, Math.toRadians(180));
           scorePose = new Pose(72, 135.25, Math.toRadians(0));
       } else {
           basePose = new Pose(105, 33, Math.toRadians(0));
           scorePose = new Pose(72, 135.25, Math.toRadians(180));
       }

        // Set starting positions
        switch (startPosition) {
            case 0:
                startPose = new Pose(22.25, 125, Math.toRadians(324));
                break;
            case 1:
                startPose = new Pose(5, 125, Math.toRadians(217));
                break;
            case 2:
                startPose = new Pose(56.75, 8.5, Math.toRadians(90));
                break;
            default:
                startPose = new Pose(87.25, 8.5, Math.toRadians(90));
                break;
        }
    }

    @Override
    public void init_loop() {
        telemetry.addLine("====DRIVER ASSIST & POSITIONING CONFIGURATION====");
        telemetry.addLine("! To enable driver assist and positioning tracking, you need to select a start position !");
        telemetry.addLine("! This configuration is optional, however, you will not be able to use these features !");
        telemetry.addLine();
        telemetry.addLine("Left Action Button: Front of the Blue Goal");
        telemetry.addLine("Top Action Button: Front of the Red Goal");
        telemetry.addLine("Bottom Action Button: Left of the Small Launch Area");
        telemetry.addLine("Right Action Button: Right of the Small Launch Area");
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

        // Initialize the flywheel, servos, and intake
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Flywheel PIDF tuning
        double p = 1.0;
        double i = 0.0;
        double d = 0.2;
        double f = 13.0;

        flywheel.setVelocityPIDFCoefficients(p, i, d, f);
        flywheel2.setVelocityPIDFCoefficients(p, i, d, f);

        // Set zero power behaviour of the flywheel
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Reverse direction
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftServo.setDirection(DcMotorSimple.Direction.REVERSE);

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
            if (flywheel.getVelocity() == 0) {
                rotateFlywheel(flywheelSpeed);
            } else {
                rotateFlywheel(0);
            }
        } else if (gamepad1.left_trigger < 0.5) {
            debounce = false;
        }

        // Check if up to speed
        if (flywheel.getVelocity() >= flywheelSpeed && !reachedVelocity) {
            gamepad1.rumble(rumbleTime); // Let driver know flywheel is up to speed
            reachedVelocity = true;
        } else if (flywheel.getVelocity() < flywheelSpeed / 2 && reachedVelocity) {
            reachedVelocity = false;
        }

        // Small Flywheel Control
        if (gamepad1.right_trigger >= 0.1 && flywheel.getVelocity() >= 0) {
            if (!intakeToggle) {
                intake.setPower(intakePower);
            }

            if (flywheel.getVelocity() > flywheelSpeed / 2) {
                rotateServos(1.0);
            }
        } else {
            if (!intakeToggle) {
                intake.setPower(0.0);
            }

            rotateServos(0.0);
        }

        // Auto Score with toggle
        if (gamepad1.aWasPressed() && teamSelected) {
            if (!autoScoring && !autoParking) {
                PathChain toScore = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), scorePose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                        .build();
                follower.followPath(toScore, true);
                autoScoring = true;
                gamepad1.rumble(rumbleTime);
            } else { // Stop AutoScore if driver hits A while AutoScore is happening
                resetStates();
            }
        }

        // Auto Park with toggle
        if (gamepad1.yWasPressed() && teamSelected) {
            if (!autoScoring && !autoParking) {
                PathChain toBase = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), basePose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), basePose.getHeading())
                        .build();
                follower.followPath(toBase, true);
                autoParking = true;
                gamepad1.rumble(rumbleTime);
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
                gamepad1.rumble(rumbleTime);
                resetStates();
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
        telemetry.addData("Flywheel Targeted Velocity", flywheelSpeed);
        telemetry.addData("Flywheel Real-Time Velocity", flywheel.getVelocity());
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
            telemetry.addLine("Top Action Button: AutoPark");
            telemetry.addLine("Bottom Action Button: AutoScore");
            Drawing.drawDebug(follower);
        }

        telemetry.update();
    }

    private void rotateFlywheel(double speed) {
        flywheel.setVelocity(speed);
        flywheel2.setVelocity(speed);
    }

    private void rotateServos(double power) {
        leftServo.setPower(power);
        rightServo.setPower(power);
    }
}