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

@Autonomous(name = "AutoMode", group = "Examples")
public class AutoMode extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private DcMotorEx flywheel;
    private CRServo servo;

    private enum Team {RED, BLUE};
    private boolean teamSelected = false;
    private Team selectedTeam = Team.RED;

    // Settings
    private double flywheelSpeed = 150.0; // Default flywheel speed
    private double pathDelay = 0.5; // Delays between steps in pathing in seconds (s)
    private double flywheelSpinUpTime = 2.0; // Time for flywheel to reach speed (s)
    private double shootingTime = 5.0; // Time to shoot all 3 balls (s)

    /*
    * 0: Front of blue goal
    * 1: Front of red goal
    * 2: Left of small launch area
    * 3: Right of small launch area
     */
    private int startPosition = 0;

    private Pose startPose, middlePose, ballsPose, ballsCapture, ballsPose2, ballsCapture2, ballsPose3, ballsCapture3, base;

    private PathChain toMiddle, topBalls, middleBalls, bottomBalls, toBase;

    private void setPosesForTeam() {
        // Set team poses based on driver input
        if (selectedTeam == Team.RED) { // Poses for Red team
            middlePose = new Pose(84, 84, Math.toRadians(45));
            ballsPose = new Pose(96, 84, Math.toRadians(0));
            ballsCapture = new Pose(120, 84, Math.toRadians(0));
            ballsPose2 = new Pose (96, 60, Math.toRadians(0));
            ballsCapture2 = new Pose(120, 60, Math.toRadians(0));
            ballsPose3 = new Pose(96, 36, Math.toRadians(0));
            ballsCapture3 = new Pose(120, 36, Math.toRadians(0));
            base = new Pose(38.65, 33.25, Math.toRadians(180));
        } else { // Poses for Blue team
            middlePose = new Pose(60, 84, Math.toRadians(135));
            ballsPose = new Pose(48, 84, Math.toRadians(180));
            ballsCapture = new Pose(24, 84, Math.toRadians(180));
            ballsPose2 = new Pose (48, 60, Math.toRadians(180));
            ballsCapture2 = new Pose(24, 60, Math.toRadians(180));
            ballsPose3 = new Pose(48, 36, Math.toRadians(180));
            ballsCapture3 = new Pose(24, 36, Math.toRadians(180));
            base = new Pose(105.25, 33.25, Math.toRadians(0));
        }

        // Set starting positions based on driver input
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

    public void buildPaths() {
        // Move to middle to shoot preloaded balls
        toMiddle = follower.pathBuilder()
                .addPath(new BezierLine(startPose, middlePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), middlePose.getHeading())
                .build();

        // Move from middle to top set of balls, intake, and return to middle
        topBalls = follower.pathBuilder()
                .addPath(new BezierLine(middlePose, ballsPose))
                .setLinearHeadingInterpolation(middlePose.getHeading(), ballsPose.getHeading())
                .addPath(new BezierLine(ballsPose, ballsCapture))
                .setLinearHeadingInterpolation(ballsPose.getHeading(), ballsCapture.getHeading())
                .addPath(new BezierLine(ballsCapture, middlePose))
                .setLinearHeadingInterpolation(ballsCapture.getHeading(), middlePose.getHeading())
                .build();

        // Move from middle to middle set of balls, intake, and return to middle
        middleBalls = follower.pathBuilder()
                .addPath(new BezierLine(middlePose, ballsPose2))
                .setLinearHeadingInterpolation(middlePose.getHeading(), ballsPose2.getHeading())
                .addPath(new BezierLine(ballsPose2, ballsCapture2))
                .setLinearHeadingInterpolation(ballsPose2.getHeading(), ballsCapture2.getHeading())
                .addPath(new BezierLine(ballsCapture2, middlePose))
                .setLinearHeadingInterpolation(ballsCapture2.getHeading(), middlePose.getHeading())
                .build();

        // Move from middle to bottom set of balls, intake, and return to middle
        bottomBalls = follower.pathBuilder()
                .addPath(new BezierLine(middlePose, ballsPose3))
                .setLinearHeadingInterpolation(middlePose.getHeading(), ballsPose3.getHeading())
                .addPath(new BezierLine(ballsPose3, ballsCapture3))
                .setLinearHeadingInterpolation(ballsPose3.getHeading(), ballsCapture3.getHeading())
                .addPath(new BezierLine(ballsCapture3, middlePose))
                .setLinearHeadingInterpolation(ballsCapture3.getHeading(), middlePose.getHeading())
                .build();

        // Move to base
        toBase = follower.pathBuilder()
                .addPath(new BezierLine(middlePose, base))
                .setLinearHeadingInterpolation(middlePose.getHeading(), base.getHeading())
                .build();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
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
        telemetry.update();
    }

    // Check for a new autonomous path
    // Check for a new autonomous path
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(toMiddle, true);
                setPathState(1);
                break;
            case 1:
                checkIfBusy(2, pathDelay);
                break;
            case 2:
                // Shoot balls
                if (shootBalls()) {
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(topBalls, true);
                setPathState(4);
                break;
            case 4:
                checkIfBusy(5, pathDelay);
                break;
            case 5:
                // Shoot balls
                if (shootBalls()) {
                    setPathState(6);
                }
                break;
            case 6:
                follower.followPath(middleBalls, true);
                setPathState(7);
                break;
            case 7:
                checkIfBusy(8, pathDelay);
                break;
            case 8:
                // Shoot balls
                if (shootBalls()) {
                    setPathState(9);
                }
                break;
            case 9:
                follower.followPath(bottomBalls, true);
                setPathState(10);
                break;
            case 10:
                checkIfBusy(11, pathDelay);
                break;
            case 11:
                // Shoot balls
                if (shootBalls()) {
                    setPathState(12);
                }
                break;
            case 12:
                follower.followPath(toBase, true);
                setPathState(13);
                break;
            case 13:
                checkIfBusy(14, pathDelay);
                break;
            case 14:
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
                if (pathTimer.getElapsedTimeSeconds() > flywheelSpinUpTime) {
                    servo.setPower(1.0);
                    shootingSubState = 2;
                    pathTimer.resetTimer();
                }
                return false;
            case 2:
                // Wait for shooting to complete
                if (pathTimer.getElapsedTimeSeconds() > shootingTime) {
                    servo.setPower(0);
                    rotateFlywheel(0);
                    shootingSubState = 0;
                    return true;
                }
                return false;
            default:
                return true;
        }
    }

    /** This  method sets the velocity of the flywheel **/
    public void rotateFlywheel(double velocity) {
        flywheel.setVelocity(velocity);
    }

    /** This method checks if the path is busy, if not: set the path state **/
    public void checkIfBusy(int state, double delay) {
        if (!follower.isBusy()) {
            if (pathTimer.getElapsedTimeSeconds() > delay) { // Delay before next part in the pathing
                setPathState(state);
            }
        }
    }

    /** This method sets the path state**/
    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setVelocityPIDFCoefficients(0,0,0,0);

        servo = hardwareMap.get(CRServo.class, "servo");
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        telemetry.addLine("====STARTING POSITION SETTINGS====");
        telemetry.addLine("! Selecting where the robot starts on the field determines its autonomous path !");
        telemetry.addLine("! Connect your controller to select a position !");
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

        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        // Make sure autonomous can't run until the driver picks a starting position
        if (!teamSelected) {
            throw new IllegalStateException("TEAM NOT SELECTED! Use the buttons on your controller during initialization to select a team before pressing play!");
        }

        // Build autonomous pathing
        setPosesForTeam();
        follower.setStartingPose(startPose);
        buildPaths();

        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}