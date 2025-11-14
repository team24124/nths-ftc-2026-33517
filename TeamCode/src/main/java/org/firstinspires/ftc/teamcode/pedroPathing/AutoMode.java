package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoMode {
}
package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
@Autonomous(name="AutoMode", group="TeamCode")
//@Disabled
public class AutoMode extends LinearOpMode {
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private final double speed = 0.5;
    private final
    int stopspeed = 0;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class,"leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class,"rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class,"leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class,"rightBackDrive");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); // Wait for the start button to be pressed

        if (opModeIsActive()) {
            leftFrontDrive.setPower(speed);
            leftBackDrive.setPower(speed);
            rightFrontDrive.setPower(speed);
            rightBackDrive.setPower(speed);
            sleep(2000);
            leftFrontDrive.setPower(stopspeed);
            leftBackDrive.setPower(stopspeed);
            rightFrontDrive.setPower(stopspeed);
            rightBackDrive.setPower(stopspeed);
        }
    }
}











