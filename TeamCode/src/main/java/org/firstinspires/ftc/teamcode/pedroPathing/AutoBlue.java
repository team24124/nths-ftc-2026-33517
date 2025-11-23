package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Auto Blue", group = "Examples")
public class AutoBlue extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(24, 120, Math.toRadians(315)),
            middlePose = new Pose(72, 72, Math.toRadians(135)),
            ballsPose = new Pose(96, 84, Math.toRadians(0)),
            ballsCapture = new Pose(120, 84, Math.toRadians(0)),
            ballsPose2 = new Pose (96, 60, Math.toRadians(0)),
            ballsCapture2 = new Pose(120, 60, Math.toRadians(0)),
            ballsPose3 = new Pose(96, 36, Math.toRadians(0)),
            ballsCapture3 = new Pose(120, 36, Math.toRadians(0)),
            base = new Pose(105, 33, Math.toRadians(0));

    private PathChain toMiddle, topBalls, middleBalls, bottomBalls, toBase;

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
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(toMiddle, true);
                setPathState(1);
                break;
            case 1:
                // TODO: Shoot preloaded balls
                checkIfBusy(2);
                break;
            case 2:
                follower.followPath(topBalls, true);
                setPathState(3);
                break;
            case 3:
                // TODO: Shoot top balls
                checkIfBusy(4);
                break;
            case 4:
                follower.followPath(middleBalls, true);
                setPathState(5);
                break;
            case 5:
                // TODO: Shoot middle balls
                checkIfBusy(6);
                break;
            case 6:
                follower.followPath(bottomBalls, true);
                setPathState(7);
                break;
            case 7:
                // TODO: Shoot bottom balls
                checkIfBusy(8);
                break;
            case 8:
                follower.followPath(toBase, true);
                setPathState(9);
                break;
            case 9:
                checkIfBusy(10);
                break;
            case 10:
                telemetry.addData("Status", "Auto Complete");
                break;
        }
    }

    /** This method checks if the path is busy, if not: set the path state **/
    public void checkIfBusy(int state) {
        if (!follower.isBusy()) {
            setPathState(state);
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
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("====WARNING====");
        telemetry.addLine("THIS AUTO CURRENTLY WORKS FROM THE FRONT OF THE BLUE GOAL");
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}