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

import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.LEDSettings;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "2020 Master Teleop", group = "Teleop")
//@Disabled
public class MasterTeleop extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();


  HDrive HDrive;
  DriftDrive driftDrive;
  FourBar FourBar;
  LEDSettings Lights;
  Gamepad gamepad;
  Hardware hardware;
  HardwareMap hardware2;
  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
    hardware = new Hardware(hardwareMap);

    driftDrive = new DriftDrive(gamepad1, hardware.leftDrive, hardware.rightDrive, hardware.centerDrive, hardware.imu, hardware.leftArm, hardware.rightArm, hardware.clampRight, hardware.clampLeft);

    HDrive = new HDrive(gamepad1, hardware.leftDrive, hardware.rightDrive, hardware.centerDrive);
    FourBar = new FourBar(gamepad2, hardware.leftArm, hardware.rightArm, hardware.clampLeft, hardware.clampRight);
    Lights = new LEDSettings(gamepad1, hardware.Lights);
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
    telemetry.addData("status", "loop test... waiting for start");
  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    runtime.reset();


  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    boolean endGame = false;
    telemetry.addData("Status", "Run Time: " + runtime.toString());
    telemetry.addData("Arrm offset: ", + FourBar.armOffset);
    telemetry.addData("Clamp offset: ", + FourBar.clampOffset);
    telemetry.addData("left ",  + FourBar.clampLeft.getPosition());
    telemetry.addData("right ",  + FourBar.clampRight.getPosition());

    HDrive.update();
    FourBar.update();
    Lights.update();






  }
}
