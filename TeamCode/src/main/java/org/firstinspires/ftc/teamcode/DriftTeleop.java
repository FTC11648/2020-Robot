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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//This teleop allows for litty drifting

@TeleOp(name="DriftTeleop", group="Pushbot")
//@Disabled
public class DriftTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware(); // Use hardware
    RobotMover robotMover;


    @Override
    public void runOpMode() {
        //I have I lot of variables
        double left;
        double right;
        double center;
        double driveVertical;
        double driveHorizontal;
        double turn;
        double driveHeading;
        double x;
        double y;
        double power;

        // Initialize the hardware variables.
        robot.init(hardwareMap);

        robotMover = new RobotMover(robot.leftDrive, robot.rightDrive, robot.centerDrive, robot.imu);

        robotMover.resetAngle();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;

            //LOTS OF MATH BEGINS
            power = Math.sqrt((x*x)+(y*y));

            driveHeading = arcTan(x, y)-Math.PI/2;
            turn = gamepad1.right_stick_x;

            driveVertical = Math.cos(Math.toRadians(robotMover.getAngle()-driveHeading))*power;
            driveHorizontal = Math.sin(Math.toRadians(robotMover.getAngle()-driveHeading))*power;

            //There are two motors on the side, so double center to compensate
            left = driveVertical + turn;
            right = driveVertical - turn;
            center = driveHorizontal*2;

            //Normalize
            if(Math.max(Math.max(left, right), center) > 1) {
                double scale = 1/Math.max(Math.max(left, right), center);
                left = left*scale;
                right = right*scale;
                center = center*scale;
            }

            // Output the safe vales to the motor drives
            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);
            robot.centerDrive.setPower(center);

            // Send telemetry message to signify robot running
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("center", "%.2f", center);
            telemetry.addData("direction", "%.2f", driveHeading);
            telemetry.update();
        }
    }

    //On a range of 0 to 2 pi
    private double arcTan(double x, double y) {
        if(x>0) {
            return Math.atan(y/x);
        }

        else if(x<0) {
            return Math.PI + Math.atan(y/x);
        }

        else if(x==0) {
            if(y>0) {
                return Math.PI/2;
            }

            if(y<0) {
                return -Math.PI/2;
            }
        }
        return 0;
    }
}

