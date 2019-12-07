package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//This opmode will drag the foundation into the build zone if you are on the BLUE team

@Autonomous(name="Foundation(BLUE)", group="Pushbot")

public class FoundationBLUE extends LinearOpMode {
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
        robotMover.encoderDrive(0.6, -31.5, -31.5, 0);
        robotMover.encoderDrive(0.6, 0, 0, 30);
    }
}