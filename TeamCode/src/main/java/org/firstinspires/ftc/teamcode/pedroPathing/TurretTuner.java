package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
@TeleOp
public class TurretTuner extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;

    private DcMotorEx flywheel, flywheel2, intake;

    /** Constants **/
    double regularSpeed = 0.80; // for regular movement speed
    double turnSpeed = 0.50; // for rotation speed
    int rumbleTime = 250; // in milliseconds

    /** Flywheel PIDF tuning **/
    int selected = 0; // 0 = flywheelSpeed, 1 = p, 2 = i, 3 = d, 4 = f
    double flywheelSpeed = 1000.0;

    double p = 0.0; // Fine tune speed
    double i = 0.0; // Fix steady state error/voltage drop
    double d = 0.0; // Dampen oscillations
    double f = 0.0; // Power to reach speed

    /** Tuning Increments **/
    double flywheelIncrement = 50.0;
    double pIncrement = 0.1;
    double iIncrement = 0.0001;
    double dIncrement = 0.1;
    double fIncrement = 0.1;

    private boolean debounce = false;
    private boolean reachedVelocity = false;

    // Intake control
    private boolean intakeToggle = false;
    private double intakePower = 1.0; // 1.0 = forward & -1.0 = reverse

    @Override
    public void init_loop() {

    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new com.pedropathing.geometry.Pose());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize the motors and servos
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

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
        super.stop();
    }

    @Override
    public void start() {
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
        follower.setTeleOpDrive(line, strafe, turn, true);

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
        if (Math.abs(flywheel.getVelocity()) >= flywheelSpeed && !reachedVelocity) {
            gamepad1.rumble(rumbleTime); // Let driver know flywheel is up to speed
            reachedVelocity = true;
        } else if (Math.abs(flywheel.getVelocity()) < flywheelSpeed - 25 && reachedVelocity) {
            reachedVelocity = false;
        }

        // Small Flywheel Control
        if (gamepad1.right_trigger >= 0.1 && Math.abs(flywheel.getVelocity()) >= 0) {
            if (!intakeToggle) {
                intake.setPower(intakePower);
            }

            if (Math.abs(flywheel.getVelocity()) > flywheelSpeed / 2) {
                //rotateServos(1.0);
            }
        } else {
            if (!intakeToggle) {
                intake.setPower(0.0);
            }

            //rotateServos(0.0);
        }

        handleTuning();
        telemetryUpdate();
    }

    /** Handles tuning control **/
    private void handleTuning() {
        boolean updatePIDF = false;

        if (gamepad1.dpadUpWasPressed()) { // Move up
            if (selected == 0) {
                selected = 4;
            } else {
                selected--;
            }
        } else if (gamepad1.dpadDownWasPressed()) { // Move down
            if (selected == 4) {
                selected = 0;
            } else {
                selected++;
            }
        } else if (gamepad1.dpadLeftWasPressed()) { // Decrease current value
            switch (selected) {
                case 0:
                    flywheelSpeed -= flywheelIncrement;

                    if (Math.abs(flywheel.getVelocity()) > 0) {
                        rotateFlywheel(flywheelSpeed);
                    }

                    break;
                case 1:
                    p -= pIncrement;
                    updatePIDF = true;
                    break;
                case 2:
                    i -= iIncrement;
                    updatePIDF = true;
                    break;
                case 3:
                    d -= dIncrement;
                    updatePIDF = true;
                    break;
                default:
                    f -= fIncrement;
                    updatePIDF = true;
                    break;
            }
        } else if (gamepad1.dpadRightWasPressed()) { // Increase current value
            switch (selected) {
                case 0:
                    flywheelSpeed += flywheelIncrement;

                    if (Math.abs(flywheel.getVelocity()) > 0) {
                        rotateFlywheel(flywheelSpeed);
                    }

                    break;
                case 1:
                    p += pIncrement;
                    updatePIDF = true;
                    break;
                case 2:
                    i += iIncrement;
                    updatePIDF = true;
                    break;
                case 3:
                    d += dIncrement;
                    updatePIDF = true;
                    break;
                default:
                    f += fIncrement;
                    updatePIDF = true;
                    break;
            }
        }

        if (updatePIDF) {
            // Update PIDF
            flywheel.setVelocityPIDFCoefficients(p, i, d, f);
            flywheel2.setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    /** This method updates the telemetry information on the driver hub/panels **/
    private void telemetryUpdate() {
        telemetry.addLine("This TeleOp is limited to basic features and is designed mainly to find optimal PIDF values");
        telemetry.addLine("You can record optimal values at the end and overwrite the ones in the TeleOpMode file. They will not save automatically");

        telemetry.addLine("====TUNING====");
        telemetry.addData("Flywheel Real-Time Velocity", Math.abs(flywheel.getVelocity()));
        telemetry.addData((selected == 0 ? "> Targeted Velocity" : "Targeted Velocity"), flywheelSpeed);
        telemetry.addData((selected == 1 ? "> P" : "P"), p);
        telemetry.addData((selected == 2 ? "> I" : "I"), i);
        telemetry.addData((selected == 3 ? "> D" : "D"), d);
        telemetry.addData((selected == 4 ? "> F" : "F"), f);

        telemetry.addLine("\n====ROBOT INFO====");
        telemetry.addData("Intake Status", (intake.getPower()) == 0 ? "Off" : "On");
        telemetry.addData("Intake Direction", (intakePower >= 0.0 ? "Forward" : "Reversed"));

        telemetry.addLine("\n====HOW TO TUNE====");
        telemetry.addLine("Use the Up & Down D-Pad buttons to navigate through the values");
        telemetry.addLine("Use the Left & Right D-Pad buttons to decrease/increase values");

        telemetry.update();
    }

    private void rotateFlywheel(double speed) {
        flywheel.setVelocity(speed);
        flywheel2.setVelocity(speed);
    }
}