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
public class BlueToBridge extends LinearOpMode {
    Hardware robot = new Hardware(); // Use hardware
    private ElapsedTime     runtime = new ElapsedTime();
    public Orientation lastAngles = new Orientation();
    public double globalAngle;

    static final double     COUNTS_PER_MOTOR_REV    = 510 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415926535897932384626433832795);


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.centerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status: ", "Waiting for start");
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.centerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.centerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        encoderDrive(31.5, 31.5, 0, 20);
        //grab
        rotate(104.15);
        encoderDrive(55, 55, 0, 20);
        rotate(-14.15);
        //let go


        //Do the course
    }

    private void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;

    }

    private double getAngle() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void rotate(double degrees) {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        double k = 0.025;

        double error = (degrees - getAngle())*k;

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        //if (degrees < 0) {
        // turn right.
        leftPower = -error;
        rightPower = error;
        //}
        // else if (degrees > 0) {
        // turn left.
        //leftPower = -power;
        //rightPower = power;
        //}
        //else return;

        // set power to rotate.
        robot.leftDrive.setPower(leftPower);
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);
        robot.rightDrive.setPower(rightPower);



        // rotate until turn is completed.
        /*if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }*/

        // left turn.

        runtime.reset();
        while (opModeIsActive() && Math.abs(getAngle()-degrees) <= 1.5) {
            error = (degrees - getAngle())*k;

            leftPower = -error;
            rightPower = error;

            robot.leftDrive.setPower(leftPower);
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
            robot.rightDrive.setPower(rightPower);

            telemetry.addData("Angle: ", "%7f", getAngle());
            telemetry.addData("Power: ", "%7f", error);
            telemetry.update();
        }

        // turn the motors off.
        robot.leftDrive.setPower(0);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.rightDrive.setPower(0);
        telemetry.addData("Angle: ", "%7f", getAngle()); telemetry.update();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void encoderDrive(double leftInches, double rightInches, double centerInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newCenterTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newCenterTarget = robot.centerDrive.getCurrentPosition() + (int)(centerInches * COUNTS_PER_INCH);

            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.centerDrive.setTargetPosition(newCenterTarget);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.centerDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int forwardError = robot.leftDrive.getCurrentPosition() - newLeftTarget;
            int centerError = robot.centerDrive.getCurrentPosition() - newCenterTarget;

            double k = 0.025;

            double forwardPower = forwardError * k;
            double centerPower = centerError * k;

            robot.leftDrive.setPower(forwardPower);
            robot.rightDrive.setPower(forwardPower);
            robot.centerDrive.setPower(centerPower);

            // Turn On RUN_TO_POSITION


            // reset the timeout time and start motion.
            runtime.reset();
            /*robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));*/



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
                forwardError = robot.leftDrive.getCurrentPosition() - newLeftTarget;
                centerError = robot.centerDrive.getCurrentPosition() - newCenterTarget;

                forwardPower = forwardError * k;
                centerPower = centerError * k;

                robot.leftDrive.setPower(forwardPower);
                robot.rightDrive.setPower(forwardPower);
                robot.centerDrive.setPower(centerPower);

            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.centerDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.centerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move



        }
    }
}
