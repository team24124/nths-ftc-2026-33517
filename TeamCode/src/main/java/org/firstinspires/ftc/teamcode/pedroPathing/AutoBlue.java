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
public class Autoblue extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(24, 24, Math.toRadians(315)),
            middlePose = new Pose(72, 72, Math.toRadians(45)),
            ballsPose = new Pose(108, 36, Math.toRadians(0)),
            ballsPose2 = new Pose (108, 60, Math.toRadians(0)),
            ballsPose3 = new Pose(108, 84, Math.toRadians(0));

    private Path move1, move2, move3, move4, move5, move6;

    public void buildPaths() {
        // Move from the front of the blue goal to the balls on the right side
        move1 = new Path(new BezierLine(startPose, ballsPose));
        move1.setLinearHeadingInterpolation(startPose.getHeading(), ballsPose.getHeading());

        // Move to the middle
        move2 = new Path(new BezierLine(ballsPose, middlePose));
        move2.setLinearHeadingInterpolation(ballsPose.getHeading(), middlePose.getHeading());

        // Move from the middle to the second set of balls
        move3 = new Path(new BezierLine(middlePose, ballsPose2));
        move3.setLinearHeadingInterpolation(middlePose.getHeading(), ballsPose2.getHeading());

        // Move from the second set of balls to the middle
        move4 = new Path(new BezierLine(ballsPose2, middlePose));
        move4.setLinearHeadingInterpolation(ballsPose2.getHeading(), middlePose.getHeading());

        // Move from the middle to the third set of balls
        move5 = new Path(new BezierLine(middlePose, ballsPose3));
        move5.setLinearHeadingInterpolation(middlePose.getHeading(), ballsPose3.getHeading());

        // Move from the third set of balls to the middle
        move6 = new Path(new BezierLine(ballsPose3, middlePose));
        move6.setLinearHeadingInterpolation(ballsPose3.getHeading(), middlePose.getHeading());
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
                // Move robot to the balls on the right side
                follower.followPath(move1);
                checkIfBusy(1);
                break;
            case 1:
                // Wait for the path to complete
                checkIfBusy(2);
                break;
            case 2:
                // Move robot to the middle of the goal facing towards the blue goal
                follower.followPath(move2);
                checkIfBusy(3);
                break;
            case 3:
                // Wait for the path to complete
                checkIfBusy(4);
                break;
            case 4:
                // Update the telemetry
                telmetry.addData("Status", "Auto Complete");
                break;
        }
    }

    public void checkIfBusy(int state) {
        if (!follower.isBusy()) {
            setPathState(state);
        }
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