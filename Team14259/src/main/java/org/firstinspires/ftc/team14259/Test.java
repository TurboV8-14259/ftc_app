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

package org.firstinspires.ftc.team14259;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;

/*
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
*/

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="Test", group="Linear Opmode")

public class Test extends LinearOpMode implements OnOpEventListener {

    // USE phone camera for navigation
    private static final boolean bUseVuforiaNav = false;

    // Whether to search for the sampling gold mineral
    private static final boolean bDoScanForGold = false;

/*
    // Declare OpMode members.
    private DigitalChannel touch;
    private ColorSensor colorSensor;
    private DistanceSensor rangeSensor;
*/

    // Timer
    private ElapsedTime runtime = new ElapsedTime();

    // GyroBot
    DriveByGyro gyroBot = new DriveByGyro();

    // TFOD
    TensorFlowObjectDetect tfod = new TensorFlowObjectDetect();

    // VuforiaNav
    VuforiaNavRoverRuckus vuforiaNav = new VuforiaNavRoverRuckus();

    // Callback for other classes who do not have access to opModeIsActive()
    @Override
    public boolean onOpIsActiveEvent() {
        return opModeIsActive();
        //return true;
    }

    // Callback for other classes who do not have access to telemetry.addData
    @Override
    public void onTelemetryAddData(String sDataItem, String sData){
        telemetry.addData(sDataItem, sData);
    }

    // Callback for other classes who do not have access to onTelemetryUpdate
    @Override
    public void onTelemetryUpdate() {
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize different components
        gyroBot.init(hardwareMap);
        if (bDoScanForGold) tfod.init(hardwareMap);
        if (bUseVuforiaNav) vuforiaNav.init(hardwareMap);

        // Enable different components to use callback (opModeIsActive and telemetry functions)
        OnOpEventListener mListener = new Test();
        gyroBot.registerOnOpEventListener(mListener);
        if (bDoScanForGold) tfod.registerOnOpEventListener(mListener);
        if (bUseVuforiaNav) vuforiaNav.registerOnOpEventListener(mListener);

/*
        touch             = hardwareMap.digitalChannel.get(       "touch");         //CHECK
        colorSensor       = hardwareMap.get(ColorSensor.class,    "sensor_color");  //CHECK
        rangeSensor       = hardwareMap.get(DistanceSensor.class, "sensor_range");  //CHECK

        // Cast to Rev2mDistanceSensor for adde methods associated with Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)rangeSensor;
*/

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

/*
        // Test callbacks
        while (opModeIsActive()) {
            telemetry.addData("Mode", gyroBot.opModeIsActive() ? "True" : "False");
            telemetry.update();
            gyroBot.telemetryAddData("AddData", "OK");
            gyroBot.telemetryUpdate();
        }
*/

/*
        // Test Touch Sensor
        boolean bPressed;
        while (opModeIsActive()) {
            bPressed = !touch.getState();
            if (bPressed) {
                telemetry.addData("Touch", bPressed ? "Pressed" : "Not pressed");
            } else {
                telemetry.addData("Touch", bPressed ? "Pressed" : "Not pressed");
            }
            telemetry.update();
        }
*/

/*
        // Test color sensor
        // Set the LED on
        float hsvValues[] = {0F,0F,0F};
        boolean bLedOn = true;
        boolean bPrevState = false;
        boolean bCurrState = false;

        colorSensor.enableLed(bLedOn);
        while (opModeIsActive()) {
            bCurrState = gamepad1.x;
            if (bCurrState && (bCurrState != bPrevState)) {

                // button is transitioning to a pressed state. So Toggle LED
                bLedOn = !bLedOn;
                colorSensor.enableLed(bLedOn);
            }
            bPrevState = bCurrState;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red()*8, colorSensor.green()*8, colorSensor.blue()*8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
        }
*/

/*
        // Test distance sensor
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("deviceName", rangeSensor.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", rangeSensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", rangeSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", rangeSensor.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", rangeSensor.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
        }
*/

/*
       while(opModeIsActive()) {
           int maxTimeLimit = 5;
           int goldSamplePosition = tfod.detect(maxTimeLimit);
           if (goldSamplePosition == tfod.GOLD_AT_LEFT) {
               telemetry.addData("Gold Mineral Position", "Left");
           } else if (goldSamplePosition == tfod.GOLD_AT_RIGHT) {
               telemetry.addData("Gold Mineral Position", "Right");
           } else {
               telemetry.addData("Gold Mineral Position", "Center");
           }
           telemetry.update();
       }
*/

/*
      if (opModeIsActive()) {
           // RightLift
           if (false)
           {
               gyroBot.robot.rightLift.setPower(0.5);  
               sleep(2000);                           

               // Stop and wait for stabilization
               gyroBot.robot.rightLift.setPower(0.0);  
               sleep(200);  
                          
               // RightLift
               gyroBot.robot.rightLift.setPower(-0.5);  
               sleep(2000);                           

              // Stop and wait for stabilization
              gyroBot.robot.rightLift.setPower(0.0);  
              sleep(200); 
          }   

          // SwingArm  
          if (false)                      
           {
              // Open Grabbers to allow arm to swing
              gyroBot.robot.openGrbbers();
              sleep(1000);
              
              // Rotate the arm to make sure it just passes the 90 degree
              gyroBot.robot.swingArm.setPower(0.5); 
              sleep(1500);                          


              // Stop the arm
              gyroBot.robot.swingArm.setPower(0);

              // Rotate the arm to make sure it just passes the 90 degree
              gyroBot.robot.swingArm.setPower(-0.5); 
              sleep(1500);                          


              // Stop the arm
              gyroBot.robot.swingArm.setPower(0);
          }   
          
          // Servo
          if (false)
          {
              gyroBot.robot.openGrbbers();
              sleep(500);
              gyroBot.robot.closeGrbbers();
          }

         // Left, Right Motor
         if (false)
         {
              gyroBot.robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
              gyroBot.robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
              gyroBot.robot.leftDrive.setPower(0.8);
              sleep(1000);
              gyroBot.robot.leftDrive.setPower(0.0);
              sleep(1000);
              gyroBot.robot.leftDrive.setPower(-0.8);
              sleep(1000);
              gyroBot.robot.leftDrive.setPower(0.0);
              sleep(1000);

              gyroBot.robot.rightDrive.setPower(0.8);
              sleep(1000);
              gyroBot.robot.rightDrive.setPower(0.0);
              sleep(1000);
              gyroBot.robot.rightDrive.setPower(-0.8);
              sleep(1000);
              gyroBot.robot.rightDrive.setPower(0.0);
              sleep(1000);

              gyroBot.robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
              gyroBot.robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         }   
      }
*/

/*
        // Test gyro drive
        if (opModeIsActive()) {
            // Reset imu heading angle
            gyroBot.gyroResetAngle();
            gyroBot.gyroDrive(gyroBot.DRIVE_SPEED,  48.0,  0.0);   // Drive FWD 48 inches
            gyroBot.gyroTurn( gyroBot.TURN_SPEED,  -45.0);         // Turn  CW to -45 Degrees
            //gyroBot.gyroHold( gyroBot.TURN_SPEED,  -45.0,  0.5);   // Hold -45 Deg heading for a 1/2 second
            //gyroBot.gyroDrive(gyroBot.DRIVE_SPEED,  12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
            //gyroBot.gyroTurn( gyroBot.TURN_SPEED,  45.0);          // Turn  CCW  to  45 Degrees
            //gyroBot.gyroHold( gyroBot.TURN_SPEED,  45.0,  0.5);    // Hold  45 Deg heading for a 1/2 second
            //gyroBot.gyroTurn( gyroBot.TURN_SPEED,  0.0);           // Turn  CW  to   0 Degrees
            //gyroBot.gyroHold( gyroBot.TURN_SPEED,  0.0,   1.0);    // Hold  0 Deg heading for a 1 second
            //gyroBot.gyroDrive(gyroBot.DRIVE_SPEED, -48.0, 0.0);    // Drive REV 48 inches
        }
*/

        // Show the elapsed game time and gold sample position.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        //sleep(2000);
    }
}
