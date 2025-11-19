package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Forward Test", group = "Pedro Pathing")
public class ForwardTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    // Start Pose
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    // End Pose
    private final Pose forwardPose = new Pose(55, 0, Math.toRadians(0));

    private Path forwardPath;

    public void buildPaths() {
        /* Simple forward path using BezierLine (straight line) */
        forwardPath = new Path(new BezierLine(startPose, forwardPose));
        forwardPath.setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start following the forward path
                follower.followPath(forwardPath);
                setPathState(1);
                break;
            case 1:
                // Wait until path is complete
                if (!follower.isBusy()) {
                    // Path complete, stop autonomous
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addLine("====AUTONOMOUS DESCRIPTION====");
        telemetry.addLine("This autonomous feature moves the robot forward a little and stops");

        telemetry.update();
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

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}