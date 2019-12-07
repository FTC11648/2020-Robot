package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//This opmode will drag the foundation into the build zone if you are on the BLUE team

@Autonomous
public class BlockToBridgeRED extends LinearOpMode {
    Hardware robot = new Hardware(); // Use hardware
    RobotMover robotMover;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robotMover = new RobotMover(robot.leftDrive, robot.rightDrive, robot.centerDrive, robot.imu);

        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.centerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status: ", "Waiting for start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Do the course
        robotMover.encoderDrive(0.6, 31.5, 31.5, 0);
        //Grab foundation
        robotMover.rotate(104.15);
        robotMover.encoderDrive(0.6, 55, 55, 0);
        robotMover.rotate(-14.15);
    }
}
