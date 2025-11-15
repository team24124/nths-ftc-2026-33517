package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcorei.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "AutoMode Line", group = "Pedro Pathing")
public class AutoMode extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private DcMotorEx leftBigFlywheel;
    private DcMotorEx rightBigFlywheel;

    private CRServo leftSmallFlywheel, rightSmallFlywheel;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    // Forward 91 inches
    private final Pose forwardPose = new Pose(5, 0, Math.toRadians(0));
    // Rotate 180 degrees at same position
    private final Pose rotatePose = new Pose(5, 0, Math.toRadians(36.87);

    private Path forwardPath;
    private Path rotatePath;

    public void buildPaths() {
        /* Forward path - moves 91 inches forward */
        forwardPath = new Path(new BezierLine(startPose, forwardPose));
        forwardPath.setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading());

        /* Rotation path - rotates 180 degrees in place */
        rotatePath = new Path(new BezierLine(forwardPose, rotatePose));
        rotatePath.setLinearHeadingInterpolation(forwardPose.getHeading(), rotatePose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start following the forward path
                follower.followPath(forwardPath);
                setPathState(1);
                break;
            case 1:
                // Wait until forward path is complete
                if (!follower.isBusy()) {
                    // Forward path complete, start rotation
                    setPathState(2);
                }
                break;
            case 2:
                // Start following the rotation path
                follower.followPath(rotatePath);
                setPathState(3);
                break;
            case 3:
                // Wait until rotation is complete
                if (!follower.isBusy()) {
                    // Rotation complete, stop autonomous
                    setPathState(-1);
                }
                break;
            case 4:
                //setting velocity to 1450
                leftBigFlywheel .setVelocity(1450);
                rightBigFlywheel .setVelocity(1450);
                setPathState(5);
                break;
            case 5:
                //launching the ball
                launchServo.setPosition(1)

        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addLine("====AUTONOMOUS DESCRIPTION====");
        telemetry.addLine("This autonomous moves the robot forward 91 inches then rotates 180 degrees, shoots at 1450");
        telemetry.addLine("\n=====WARNING=====");
        telemetry.addLine("This autonomous will only work from the upper left corner in front of the blue goal");
        telemetry.addData("Path State", pathState);

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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}