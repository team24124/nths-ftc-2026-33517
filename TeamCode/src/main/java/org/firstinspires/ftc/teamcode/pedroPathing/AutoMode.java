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
        private double flywheelSpeed = 3000.0; // Default flywheel speed
        private double targettedFlywheelSpeed = 1200.0;
        private double shootingTime = 5.5; // Time to shoot all 3 balls (s)
        private double intakeBotSpeed = 0.5; // 0.0-1.0 Determines speed of the BOT while collecting balls

        private DcMotorEx flywheel, flywheel2, intake;
        private CRServo servos;

        private SharedPoseStorage.Team selectedTeam = SharedPoseStorage.Team.RED;
        private boolean teamSelected = false;
        private int mode = 1; // 0 = Idle | 1 = Active

        /*
        * 0: Front of blue goal
        * 1: Front of red goal
        * 2: Left of small launch area
        * 3: Right of small launch area
         */
        private int startPosition = 0;

        private Pose startPose, middlePose, ballsPose, ballsCapture, ballsPose2, ballsCapture2, ballsPose3, ballsCapture3, lever, passivePose;

        private PathChain toMiddle, toTopBalls, captureTop, returnFromTop, toMiddleBalls, captureMiddle, returnFromMiddle, toBottomBalls, captureBottom, returnFromBottom, toLever, toPassive;

        private void setPosesForTeam() {
            // Set team poses based on driver input
            if (selectedTeam == SharedPoseStorage.Team.RED) { // Poses for Red team
                middlePose = new Pose(84, 84, Math.toRadians(43));
                ballsPose = new Pose(96, 83, Math.toRadians(0));
                ballsCapture = new Pose(144, 83, Math.toRadians(0));
                ballsPose2 = new Pose(98, 59, Math.toRadians(0));
                ballsCapture2 = new Pose(132, 59, Math.toRadians(0));
                ballsPose3 = new Pose(96, 35, Math.toRadians(0));
                ballsCapture3 = new Pose(130, 35, Math.toRadians(0));
                lever = new Pose(114, 72, Math.toRadians(0));
            } else { // Poses for Blue team
                middlePose = new Pose(60, 84, Math.toRadians(133));
                ballsPose = new Pose(48, 83, Math.toRadians(180));
                ballsCapture = new Pose(0, 83, Math.toRadians(180));
                ballsPose2 = new Pose(46, 59, Math.toRadians(180));
                ballsCapture2 = new Pose(12, 59, Math.toRadians(180));
                ballsPose3 = new Pose(48, 35, Math.toRadians(180));
                ballsCapture3 = new Pose(10, 35, Math.toRadians(180));
                lever = new Pose(30, 72, Math.toRadians(180));
            }

            // Set starting positions
            switch (startPosition) {
                case 0:
                    startPose = new Pose(22.25, 125, Math.toRadians(324));
                    break;
                case 1:
                    startPose = new Pose(121.75, 125, Math.toRadians(217));
                    break;
                case 2:
                    startPose = new Pose(56.75, 8.5, Math.toRadians(90));
                    break;
                default:
                    startPose = new Pose(87.25, 8.5, Math.toRadians(90));
                    break;
            }

            // Set passive pose
            if (mode == 0) {
                if (startPosition == 0 || startPosition == 1) {
                    if (selectedTeam == SharedPoseStorage.Team.BLUE) {
                        passivePose = new Pose(60, 132, Math.toRadians(0));
                    } else {
                        passivePose = new Pose(84, 132, Math.toRadians(180));
                    }
                } else {
                    if (selectedTeam == SharedPoseStorage.Team.BLUE) {
                        passivePose = new Pose(36, 12, Math.toRadians(90));
                    } else {
                        passivePose = new Pose(108, 12, Math.toRadians(90));
                    }
                }
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

            if (mode == 0) {
                toPassive = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, passivePose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), passivePose.getHeading())
                        .build();
            }
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
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            servos = hardwareMap.get(CRServo.class, "servos");

            // Flywheel PIDF tuning
            double p = 0.7; // Fine tune speed
            double i = 0.0; // Fix steady state error/voltage drop
            double d = 2.0; // Dampen oscillations
            double f = 5.0; // Power to reach speed

            flywheel.setVelocityPIDFCoefficients(p, i, d, f);
            flywheel2.setVelocityPIDFCoefficients(p, i, d, f);

            // Set zero power behaviour of the flywheel
            flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            // Reverse direction
            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);

            // Initialize the panels visualizer
            Drawing.init();
        }

        @Override
        public void init_loop() {
            telemetry.addLine("====STARTING POSITION SETTINGS====");
            telemetry.addLine("! Selecting where the robot starts on the field determines its autonomous path !");
            telemetry.addLine();
            telemetry.addLine("Left Action Button: Front of the Blue Goal");
            telemetry.addLine("Top Action Button: Front of the Red Goal");
            telemetry.addLine("Bottom Action Button: Left of the Small Launch Area");
            telemetry.addLine("Right Action Button: Right of the Small Launch Area");
            telemetry.addLine();
            telemetry.addLine("====MODE SETTINGS====");
            telemetry.addLine("! IDLE mode moves the bot to a passive position that does not interfere with other autos !");
            telemetry.addLine("! ACTIVE mode runs the autonomous routine !");
            telemetry.addLine();
            telemetry.addLine("Up D-Pad: Switch Modes");
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

            telemetry.addLine("MODE: " + (mode == 0 ? "Idle" : "Active"));

            // Controls to select team
            if (gamepad1.x) {
                selectedTeam = SharedPoseStorage.Team.BLUE;
                startPosition = 0;
                teamSelected = true;
            } else if (gamepad1.y) {
                selectedTeam = SharedPoseStorage.Team.RED;
                startPosition = 1;
                teamSelected = true;
            }  else if (gamepad1.a) {
                selectedTeam = SharedPoseStorage.Team.BLUE;
                startPosition = 2;
                teamSelected = true;
            } else if (gamepad1.b) {
                selectedTeam = SharedPoseStorage.Team.RED;
                startPosition = 3;
                teamSelected = true;
            }

            // Switch mode
            if (gamepad1.dpadUpWasPressed()) {
                if (mode == 1) { // Switch to idle
                    mode = 0;
                } else {
                    mode = 1; // Switch to active
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

            // Save team for TeleOp
            SharedPoseStorage.currentTeam = selectedTeam;
            SharedPoseStorage.teamAvailable = true;

            opmodeTimer.resetTimer();
            setPathState(0);
        }

        @Override
        public void loop() {
            // These loop the movements of the robot, these must be called continuously in order to work
            follower.update();
            autonomousPathUpdate();

            // Save the current pose for TeleOp
            SharedPoseStorage.currentPose = follower.getPose();
            SharedPoseStorage.poseAvailable = true;

            // Feedback to Driver Hub for debugging
            telemetry.addData("Status", "Auto in progress..");
            telemetry.addData("Path State", pathState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());
            telemetry.addData("Flywheel Targeted Velocity", targettedFlywheelSpeed);
            telemetry.addData("Flywheel Real-Time Velocity", Math.abs(flywheel.getVelocity()));
            telemetry.update();

            Drawing.drawDebug(follower);
        }

        // Check for a new autonomous path
        // TODO: Stop intake after grabbing balls in path
        public void autonomousPathUpdate() {
            switch (pathState) {
                case 0:
                    if (mode == 1) {
                        rotateFlywheel(flywheelSpeed);
                        intake.setPower(1.0);
                        servos.setPower(0.2);
                        follower.followPath(toMiddle, true);
                        setPathState(1);
                    } else {
                        setPathState(27);
                    }
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
                    setPathState(4);
                    break;
                case 4:
                    checkIfBusy(5, 0);
                    break;
                case 5:
                    follower.setMaxPower(intakeBotSpeed);
                    intake.setPower(1.0);
                    follower.followPath(captureTop, false);
                    setPathState(6);
                    break;
                case 6:
                    checkIfBusy(7, 0);
                    break;
                case 7:
                    rotateFlywheel(flywheelSpeed);
                    follower.setMaxPower(1.0);
                    follower.followPath(returnFromTop, true);
                    //servos.setPower(0.1);
                    setPathState(8);
                    break;
                case 8:
                    checkIfBusy(9, 0);
                    break;
                case 9:
                    //intake.setPower(0.0);
                    if (shootBalls()) {
                        setPathState(10);
                    }
                    break;
                case 10:
                    follower.followPath(toMiddleBalls, true);
                    setPathState(11);
                    break;
                case 11:
                    checkIfBusy(12, 0);
                    break;
                case 12:
                    follower.setMaxPower(intakeBotSpeed);
                    intake.setPower(1.0);
                    follower.followPath(captureMiddle, true);
                    setPathState(13);
                    break;
                case 13:
                    checkIfBusy(14, 0);
                    break;
                case 14:
                    rotateFlywheel(flywheelSpeed);
                    follower.setMaxPower(1.0);
                    follower.followPath(returnFromMiddle, true);
                    setPathState(15);
                    break;
                case 15:
                    checkIfBusy(16, 0);
                    break;
                case 16:
                    //intake.setPower(0.0);
                    if (shootBalls()) {
                        setPathState(17);
                    }
                    break;
                case 17:
                    follower.followPath(toBottomBalls, true);
                    setPathState(18);
                    break;
                case 18:
                    checkIfBusy(19, 0);
                    break;
                case 19:
                    follower.setMaxPower(intakeBotSpeed);
                    intake.setPower(1.0);
                    follower.followPath(captureBottom, true);
                    setPathState(20);
                    break;
                case 20:
                    checkIfBusy(21, 0);
                    break;
                case 21:
                    rotateFlywheel(flywheelSpeed);
                    follower.setMaxPower(1.0);
                    follower.followPath(returnFromBottom, true);
                    setPathState(22);
                    break;
                case 22:
                    checkIfBusy(23, 0);
                    break;
                case 23:
                    //intake.setPower(0.0);
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
                    servos.setPower(0.0);
                    flywheel.setPower(0.0);
                    intake.setPower(0.0);
                    break;
                // CASES AFTER THIS IS FOR IDLE PATHING
                case 27:
                    follower.followPath(toPassive, true);
                    setPathState(28);
                    break;
                case 28:
                    checkIfBusy(26, 0);
                    break;
            }
        }

        /** Shooter Handling **/
        private int shootingSubState = 0;
        private int ballsShot = 0;

        public boolean shootBalls() {
            switch (shootingSubState) {
                case 0:
                    // Initialize shooting
                    ballsShot = 0;
                    shootingSubState = 1;
                    return false;
                case 1:
                    // Wait for flywheel to reach speed
                    if (Math.abs(flywheel.getVelocity()) >= targettedFlywheelSpeed - 90) {
                        shootingSubState = 2;
                    }
                    return false;
                case 2:
                    // Make sure flywheel is at speed before shooting
                    if (Math.abs(flywheel.getVelocity()) >= targettedFlywheelSpeed - 90) {
                        shootingSubState = 3; // Move to actually shooting
                    }
                    return false;
                case 3:
                    // Run servos and wait for velocity drop
                    servos.setPower(1.0);
                    intake.setPower(1.0);

                    // Check if flywheel dropped (ball was shot)
                    if (Math.abs(flywheel.getVelocity()) < targettedFlywheelSpeed - 100) {
                        // Ball was shot! Stop servos and wait for recovery
                        servos.setPower(0.0);
                        ballsShot++;
                        shootingSubState = 4;
                    }
                    return false;
                case 4:
                    // Wait for flywheel to recover to speed
                    if (Math.abs(flywheel.getVelocity()) >= targettedFlywheelSpeed - 100) {
                        // Flywheel recovered - check if done or shoot next
                        if (ballsShot >= 3) {
                            // All balls shot, finish up
                            rotateFlywheel(0);
                            shootingSubState = 0;
                            return true;
                        } else {
                            // Ready for next ball - go back to case 2 to ensure flywheel is at speed
                            shootingSubState = 2;
                        }
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