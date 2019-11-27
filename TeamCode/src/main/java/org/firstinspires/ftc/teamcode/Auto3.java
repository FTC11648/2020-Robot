/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto3", group="Pushbot")

public class Auto3 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    public Orientation lastAngles = new Orientation();
    public double globalAngle;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.centerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.centerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        waitForStart();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, 31.5, 31.5, 4.0);  // S1: Forward 31.5 Inches with a 4 Sec timeout
        rotate(-104.15);
        encoderDrive(DRIVE_SPEED, 63.42, 63.42, 4.0);  // S3: Forward 63.42 Inches with a 4 Sec timeout
        rotate(-14.15); // Turn right 14.15g

       /* robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move*/

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */


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
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        double k = 0.001;

        double error = (degrees - getAngle()) * k;

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
        robot.rightDrive.setPower(rightPower);


        // rotate until turn is completed.
        /*if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}
            while (opModeIsActive() && getAngle() > degrees) {}
        }*/

        // left turn.

        runtime.reset();
        while (opModeIsActive() && Math.abs(getAngle() - degrees) <= 1.5) {
            error = (degrees - getAngle()) * k;

            leftPower = -error;
            rightPower = error;

            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);


            telemetry.addData("Angle: ", "%7f", getAngle());
            telemetry.addData("Power: ", "%7f", error);
            telemetry.update();
        }
    }
        public void encoderDrive ( double speed,
        double leftInches, double rightInches,
        double timeoutS){
            int newLeftTarget;
            int newRightTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                robot.leftDrive.setTargetPosition(newLeftTarget);
                robot.rightDrive.setTargetPosition(newRightTarget);

                // Turn On RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                int rightError = newRightTarget - robot.rightDrive.getCurrentPosition();
                int leftError = newLeftTarget - robot.leftDrive.getCurrentPosition();

                robot.rightDrive.setPower(0.001 * rightError);
                robot.leftDrive.setPower(0.001 * leftError);


                // reset the timeout time and start motion.
                runtime.reset();


                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {

                    rightError = newRightTarget - robot.rightDrive.getCurrentPosition();
                    leftError = newLeftTarget - robot.leftDrive.getCurrentPosition();

                    robot.leftDrive.setPower(0.001 * leftError);
                    robot.rightDrive.setPower(0.001 * rightError);

                    telemetry.addData("leftFrontTarget", "Running to %7d", robot.leftDrive.getTargetPosition());
                    telemetry.addData("rightFrontTarget", "Running to %7d", robot.rightDrive.getTargetPosition());
                    telemetry.addData("Encoder LEFT", "Running at %7d", robot.leftDrive.getCurrentPosition());
                    telemetry.addData("Encoder RIGHT", "Running at %7d", robot.leftDrive.getCurrentPosition());


                }

                // Stop all motion;
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.centerDrive.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        }

}