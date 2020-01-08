/*
 * Copyright (c) 2018 Craig MacFarlane
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Craig MacFarlane nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXE MPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.Timer;
import java.util.TimerTask;
import java.util.Date;
import java.util.concurrent.TimeUnit;

/*
 * Display patterns of a REV Robotics Blinkin LED Driver.
 * AUTO mode cycles through all of the patterns.
 * MANUAL mode allows the user to manually change patterns using the
 * left and right bumpers of a gamepad.
 *
 * Configure the driver on a servo port, and name it "blinkin".
 *
 * Displays the first pattern upon init.
 */
@TeleOp(name="BlinkinExample")

public class FirstLEDSettings extends OpMode {
    private boolean FInput=true;
    /*
     * Change the pattern every 10 seconds in AUTO mode.
     */
    private final static int LED_PERIOD = 100;
    ElapsedTime runtime = new ElapsedTime();
    boolean istrue=false;
    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private final static int GAMEPAD_LOCKOUT = 500;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Telemetry.Item display;
    DisplayKind displayKind;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }

    @Override
    public void init()
    {
        displayKind = DisplayKind.MANUAL; //This is used to change between the Manual and the Auto mode

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE;           //Here is where you can change the color of the Lights
        blinkinLedDriver.setPattern(pattern);
        display = telemetry.addData("Display Kind: ", displayKind.toString());
        patternName = telemetry.addData("Pattern: ", pattern.toString());

        //ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
        //gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
    }

    @Override
    public void loop() //Be careful with the Loop method
    {
         /*
        if((gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y || gamepad1.left_bumper || gamepad1.right_bumper)&& FInput){  (This was a failed way for using the LED with the button presses!)
    blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;           //Colors which change when input occurs
    blinkinLedDriver.setPattern(pattern);
    this.FInput=false;
    runtime.reset();
    istrue=true;
}
if(runtime.seconds() > 5 && istrue){
    pattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;           //change pattern when enter the deathmatch (Can change # of sunconds);
    blinkinLedDriver.setPattern(pattern);
}

       */
        handleGamepad(); //Colors switch due to Gamepad touch

        if (displayKind == DisplayKind.AUTO) {
            doAutoDisplay();
        } else {
            /*
             * MANUAL mode: Nothing to do, setting the pattern as a result of a gamepad event.
             */
        }

}

    /*
     * handleGamepad
     *
     * Responds to a gamepad button press.  Demonstrates rate limiting for
     * button presses.  If loop() is called every 10ms and and you don't rate
     * limit, then any given button press may register as multiple button presses,
     * which in this application is problematic.
     *
     * A: Manual mode, Right bumper displays the next pattern, left bumper displays the previous pattern.
     * B: Auto mode, pattern cycles, changing every LED_PERIOD seconds.
     */
    protected void handleGamepad()
    {
        /*if (!gamepadRateLimit.hasExpired()) {
            return;
        }*/

        if (gamepad1.a && FInput) {     //This should check for input from the Gamepad and if it is detected then it will switch the colors while starting the Timer!
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
            displayPattern();
            handleTimer();
            this.FInput=false;
           // gamepadRateLimit.reset();
        } else if (gamepad1.b && FInput) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
            displayPattern();
            handleTimer();
            this.FInput=false;
            //gamepadRateLimit.reset();
        } else if (gamepad1.left_bumper && FInput) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
           displayPattern();
           handleTimer();
           this.FInput=false;
           //gamepadRateLimit.reset();
        } else if (gamepad1.right_bumper && FInput) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
        displayPattern();
        handleTimer();
        this.FInput=false;
        //gamepadRateLimit.reset();
        }else if (gamepad1.x && FInput){
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
        displayPattern();
        handleTimer();
        this.FInput=false;
        //gamepadRateLimit.reset();
        }else if (gamepad1.y && FInput){
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
            displayPattern();
            handleTimer();
            this.FInput=false;
          //  gamepadRateLimit.reset();
        }
    }

    protected void setDisplayKind(DisplayKind displayKind)
    {
        this.displayKind = displayKind;
        display.setValue(displayKind.toString());
    }

    protected void doAutoDisplay()
    {
        if (ledCycleDeadline.hasExpired()) {
            pattern = pattern.next();
            displayPattern();
            ledCycleDeadline.reset();
        }
    }

    protected void displayPattern()
    {
        blinkinLedDriver.setPattern(pattern);
        patternName.setValue(pattern.toString());
    }
protected void handleTimer(){
/*    int interval = 10000; // This is in milli seconds! (10 seconds)pattern = RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE;
           displayPattern();
    Date timeToRun = new Date(System.currentTimeMillis() + interval);
    Timer timer = new Timer();

    timer.schedule(new TimerTask() {
        public void run() {
            pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
            displayPattern();                    //This is executed after the time has expired!
        }
    }, timeToRun);


*/
}
}

