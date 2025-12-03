package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "AutoMode", group = "Examples")
public class AutoMode extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Constants
    private double flywheelSpeed = 1600.0; // Default flywheel speed
    private double shootingTime = 6.0; // Time to shoot all 3 balls (s)

    private DcMotorEx flywheel, flywheel2, intake;
    private CRServo leftServo, rightServo;

    private enum Team {RED, BLUE};
    private boolean teamSelected = false;
    private Team selectedTeam = Team.RED;

    /*
    * 0: Front of blue goal
    * 1: Front of red goal
    * 2: Left of small launch area
    * 3: Right of small launch area
     */
    private int startPosition = 0;

    private Pose startPose, middlePose, ballsPose, ballsCapture, ballsPose2, ballsCapture2, ballsPose3, ballsCapture3, lever;

    private PathChain toMiddle, toTopBalls, captureTop, returnFromTop, toMiddleBalls, captureMiddle, returnFromMiddle, toBottomBalls, captureBottom, returnFromBottom, toLever;

    private void setPosesForTeam() {
        // Set team poses based on driver input
        if (selectedTeam == Team.RED) { // Poses for Red team
            middlePose = new Pose(84, 84, Math.toRadians(42));
            ballsPose = new Pose(96, 84, Math.toRadians(0));
            ballsCapture = new Pose(120, 84, Math.toRadians(0));
            ballsPose2 = new Pose (96, 60, Math.toRadians(0));
            ballsCapture2 = new Pose(120, 60, Math.toRadians(0));
            ballsPose3 = new Pose(96, 36, Math.toRadians(0));
            ballsCapture3 = new Pose(120, 36, Math.toRadians(0));
            lever = new Pose(120, 72, Math.toRadians(0));
        } else { // Poses for Blue team
            middlePose = new Pose(60, 84, Math.toRadians(138));
            ballsPose = new Pose(48, 84, Math.toRadians(180));
            ballsCapture = new Pose(22, 84, Math.toRadians(180));
            ballsPose2 = new Pose (48, 59, Math.toRadians(180));
            ballsCapture2 = new Pose(18, 59, Math.toRadians(180));
            ballsPose3 = new Pose(48, 38, Math.toRadians(180));
            ballsCapture3 = new Pose(17, 38, Math.toRadians(180));
            lever = new Pose(24, 72, Math.toRadians(180));
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

    public void buildPaths() {
        toMiddle = follower.pathBuilder()
                .addPath(new BezierLine(startPose, middlePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), middlePose.getHeading())
                .build();

        toTopBalls = follower.pathBuilder()
                .addPath(new BezierLine(middlePose, ballsPose))
                .setLinearHeadingInterpolation(middlePose.getHeading(), ballsPose.getHeading())
                .build();

        captureTop = follower.pathBuilder()
                .addPath(new BezierLine(ballsPose, ballsCapture))
                .setLinearHeadingInterpolation(ballsPose.getHeading(), ballsCapture.getHeading())
                .build();

        returnFromTop = follower.pathBuilder()
                .addPath(new BezierLine(ballsCapture, middlePose))
                .setLinearHeadingInterpolation(ballsCapture.getHeading(), middlePose.getHeading())
                .build();

        toMiddleBalls = follower.pathBuilder()
                .addPath(new BezierLine(middlePose, ballsPose2))
                .setLinearHeadingInterpolation(middlePose.getHeading(), ballsPose2.getHeading())
                .build();

        captureMiddle = follower.pathBuilder()
                .addPath(new BezierLine(ballsPose2, ballsCapture2))
                .setLinearHeadingInterpolation(ballsPose2.getHeading(), ballsCapture2.getHeading())
                .build();

        returnFromMiddle = follower.pathBuilder()
                .addPath(new BezierLine(ballsCapture2, middlePose))
                .setLinearHeadingInterpolation(ballsCapture2.getHeading(), middlePose.getHeading())
                .build();

        toBottomBalls = follower.pathBuilder()
                .addPath(new BezierLine(middlePose, ballsPose3))
                .setLinearHeadingInterpolation(middlePose.getHeading(), ballsPose3.getHeading())
                .build();

        captureBottom = follower.pathBuilder()
                .addPath(new BezierLine(ballsPose3, ballsCapture3))
                .setLinearHeadingInterpolation(ballsPose3.getHeading(), ballsCapture3.getHeading())
                .build();

        returnFromBottom = follower.pathBuilder()
                .addPath(new BezierLine(ballsCapture3, middlePose))
                .setLinearHeadingInterpolation(ballsCapture3.getHeading(), middlePose.getHeading())
                .build();

        toLever = follower.pathBuilder()
                .addPath(new BezierLine(middlePose, lever))
                .setLinearHeadingInterpolation(middlePose.getHeading(), lever.getHeading())
                .build();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        // Initialize the motors and servos
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Flywheel PIDF tuning
        double p = 1.0;
        double i = 0.0;
        double d = 0.0;
        double f = 12.5;

        flywheel.setVelocityPIDFCoefficients(p, i, d, f);
        flywheel2.setVelocityPIDFCoefficients(p, i, d, f);

        // Set zero power behaviour of the flywheel
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Reverse direction
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the panels visualizer
        Drawing.init();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("====STARTING POSITION SETTINGS====");
        telemetry.addLine("! Selecting where the robot starts on the field determines its autonomous path !");
        telemetry.addLine("! Connect your controller to select a position !");
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

        telemetry.update();
    }

    @Override
    public void start() {
        // Make sure autonomous can't run until the driver picks a starting position
        if (!teamSelected) {
            throw new IllegalStateException("START POSITION NOT SELECTED! Use the buttons on your controller during initialization to select a position before pressing play!");
        }

        // Build autonomous pathing
        setPosesForTeam();
        follower.setStartingPose(startPose);
        buildPaths();

        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("Status", "Auto in progress..");
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Flywheel Targeted Velocity", flywheelSpeed);
        telemetry.addData("Flywheel Real-Time Velocity", flywheel.getVelocity());
        telemetry.update();

        Drawing.drawDebug(follower);
    }

    // Check for a new autonomous path
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(toMiddle, true);
                setPathState(1);
                break;
            case 1:
                checkIfBusy(2, 0);
                break;
            case 2:
                if (shootBalls()) {
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(toTopBalls, true);
                intake.setPower(1.0);
                setPathState(4);
                break;
            case 4:
                checkIfBusy(5, 0);
                break;
            case 5:
                follower.followPath(captureTop, true);
                setPathState(6);
                break;
            case 6:
                checkIfBusy(7, 0);
                break;
            case 7:
                intake.setPower(0.0);
                follower.followPath(returnFromTop, true);
                setPathState(8);
                break;
            case 8:
                checkIfBusy(9, 0);
                break;
            case 9:
                if (shootBalls()) {
                    setPathState(10);
                }
                break;
            case 10:
                follower.followPath(toMiddleBalls, true);
                intake.setPower(1.0);
                setPathState(11);
                break;
            case 11:
                checkIfBusy(12, 0);
                break;
            case 12:
                follower.followPath(captureMiddle, true);
                setPathState(13);
                break;
            case 13:
                checkIfBusy(14, 0);
                break;
            case 14:
                intake.setPower(0.0);
                follower.followPath(returnFromMiddle, true);
                setPathState(15);
                break;
            case 15:
                checkIfBusy(16, 0);
                break;
            case 16:
                if (shootBalls()) {
                    setPathState(17);
                }
                break;
            case 17:
                follower.followPath(toBottomBalls, true);
                intake.setPower(1.0);
                setPathState(18);
                break;
            case 18:
                checkIfBusy(19, 0);
                break;
            case 19:
                follower.followPath(captureBottom, true);
                setPathState(20);
                break;
            case 20:
                checkIfBusy(21, 0);
                break;
            case 21:
                intake.setPower(0.0);
                follower.followPath(returnFromBottom, true);
                setPathState(22);
                break;
            case 22:
                checkIfBusy(23, 0);
                break;
            case 23:
                if (shootBalls()) {
                    setPathState(24);
                }
                break;
            case 24:
                follower.followPath(toLever, true);
                setPathState(25);
                break;
            case 25:
                checkIfBusy(26, 0);
                break;
            case 26:
                telemetry.addData("Status", "Auto Complete");
                break;
        }
    }

    /** This method handles the shooting **/
    private int shootingSubState = 0;

    public boolean shootBalls() {
        switch (shootingSubState) {
            case 0:
                // Charge up the flywheel
                rotateFlywheel(flywheelSpeed);
                shootingSubState = 1;
                pathTimer.resetTimer();
                return false;
            case 1:
                // Wait for flywheel to reach speed
                if (flywheel.getVelocity() >= flywheelSpeed) {
                    leftServo.setPower(1.0);
                    rightServo.setPower(1.0);
                    shootingSubState = 2;
                    pathTimer.resetTimer();
                }
                return false;
            case 2:
                // Wait for shooting to complete
                if (pathTimer.getElapsedTimeSeconds() > shootingTime) {
                    leftServo.setPower(0.0);
                    rightServo.setPower(0.0);
                    rotateFlywheel(0);
                    shootingSubState = 0;
                    return true;
                }
                return false;
            default:
                return true;
        }
    }

    /** Rotates flywheel **/
    private void rotateFlywheel(double speed) {
        flywheel.setVelocity(speed);
        flywheel2.setVelocity(speed);
    }

    /** Rotates servos **/
    private void rotateServos(double power) {
        leftServo.setPower(power);
        rightServo.setPower(power);
    }

    /** Checks if the path is busy, if not: set the path state **/
    public void checkIfBusy(int state, double delay) {
        if (!follower.isBusy()) {
            if (pathTimer.getElapsedTimeSeconds() > delay) { // Delay if necessary
                setPathState(state);
            }
        }
    }

    /** Sets the path state **/
    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }
}