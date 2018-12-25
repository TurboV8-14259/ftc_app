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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;

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
@Autonomous(name="Crater", group="Linear Opmode")

public class Crater extends LinearOpMode implements OnOpEventListener {

    // USE phone camera for navigation
    private static final boolean bUseVuforiaNav = false;

    // Whether to search for the sampling gold mineral
    private static final boolean bDoScanForGold = false;

    // Timer
    private ElapsedTime    runtime = new ElapsedTime();

    // GyroBot
    DriveByGyro gyroBot = new DriveByGyro();

    // TFOD
    TensorFlowObjectDetect tfod = new TensorFlowObjectDetect();

    // VuforiaNav
    VuforiaNavRoverRuckus vuforiaNav = new VuforiaNavRoverRuckus();

    // Callback for other classes who do not have access to opModeIsActive()
    @Override
    public boolean onOpIsActiveEvent() {
        //return opModeIsActive();
        return true;
    }

    // Callback for other classes who do not have access to telemetry.addData
    @Override
    public void onTelemetryAddData(String sDataItem, String sData) {
        //telemetry.addData(sDataItem, sData);
    }

    // Callback for other classes who do not have access to onTelemetryUpdate
    @Override
    public void onTelemetryUpdate() {
        //telemetry.update();
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
        OnOpEventListener mListener = new Crater();
        gyroBot.registerOnOpEventListener(mListener);
        if (bDoScanForGold) tfod.registerOnOpEventListener(mListener);
        if (bUseVuforiaNav) vuforiaNav.registerOnOpEventListener(mListener);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

/*
         // Test gyro drive
         if (opModeIsActive()) {
             // Reset imu heading angle
             gyroBot.gyroResetAngle();
             gyroBot.gyroTurn( gyroBot.TURN_SPEED,  -45.0);        // Turn  CW  to -45 Degrees
             gyroBot.gyroTurn( gyroBot.TURN_SPEED,  0.0);          // Turn  CW  to   0 Degrees
             gyroBot.gyroTurn( gyroBot.TURN_SPEED,  45.0);         // Turn  CCW to  45 Degrees
             gyroBot.gyroTurn( gyroBot.TURN_SPEED,  0.0);          // Turn  CW  to   0 Degrees
             gyroBot.gyroDrive(gyroBot.DRIVE_SPEED,  48.0,  0.0);  // Drive FWD 48 inches
             gyroBot.gyroDrive(gyroBot.DRIVE_SPEED, -48.0, 0.0);   // Drive REV 48 inches
         }
*/

        // if bScanForGold is false, goldSamplePosition defines the gold sampling position.
        int goldSamplePosition = tfod.GOLD_AT_CENTER;
        //int goldSamplePosition = tfod.GOLD_AT_LEFT;
        //int goldSamplePosition = tfod.GOLD_AT_RIGHT;

        // When tuning, enable the following one by one in order by changing it to true
        boolean bDoLanding            = true;
        boolean bGetIntoStartingPoint = true;
        boolean bDoSampling           = true;
        boolean bDoClaiming           = true;
        boolean doParking             = true;

        // Reset imu heading angle
        gyroBot.gyroResetAngle();

        // Scan for gold mineral position among 3 sampling mineral for a few seconds
        double maxTimeLimit = 5; // 5 seconds
        if (bDoScanForGold && opModeIsActive()) {
            goldSamplePosition = tfod.detect(maxTimeLimit);
        }

        // Land robot to the ground
        if (bDoLanding && opModeIsActive()) {
            // Lower the robot a little first with less power
            gyroBot.robot.rightLift.setPower(0.6);  //CHECK
            sleep(2000);                            //CHECK

            // Stop and wait for stabilization
            gyroBot.robot.rightLift.setPower(0.0);
            sleep(200);                            //CHECK
        }

        // Starting point: the corner of the tile that contains 3 sampling minerals, closet to the lander
        // 0.  drive to the starting point and turn to a correct direction
        //     a. correct the angle caused by landing
        //     b. correct the dislocation caused by landing
        //     c. face to the sampling mineral in the middle
        //     d. drive to the starting point
        // 1.  do sampling
        //     a. rotate to face the gold sampling mineral
        //     b. drive to the gold sampling mineral
        // 2.  do claiming
        //     a. drive back to the starting point
        //     b. turn right to face the wall between our crater and opponent's depot (45 degree)
        //     c. drive backward to the last tile next to the wall
        //     d. turn left to face our crater to make robot parallel to the wall
        //     e. drive straight back to depot
        //     f. drop team marker
        // 3. do parking
        //     a. drive straigt forward to crater
        //     b. put the arm into crater

        double speed    = 0.0;
        double angle    = 0.0;
        double distance = 0.0;

        // 0. drive to the starting point and turn to a correct direction
        if (bGetIntoStartingPoint) {

            // Assume the robot heading is parallel to the lander side surface: 45deg, CCW to North
            // a. correct the angle caused by landing to make sure robot is parallel to the
            //    lander side surface.
            if (false)
            {
                speed = 0.5 * gyroBot.TURN_SPEED;      //half speed for small move
                //angle = -1.0 * gyroBot.gyroGetAngle();
                //gyroBot.gyroResetAngle();
                //gyroBot.gyroTurn(speed, angle);
                //gyroBot.gyroResetAngle();
                angle = 0.0;
                gyroBot.gyroTurn(speed, angle);
            }

            // b. correct the dislocation caused by landing
            //    make sure the center of robot is on the diagonal line in the landing title
            if (false)
            {
                speed    = 0.5 * gyroBot.DRIVE_SPEED; //half speed for small move
                angle    = angle;                     //No change
                distance = 3.0;                       //CHECK: forward 3 inches
                gyroBot.gyroDrive(speed, distance, angle);
            }

            // c. face to the starting point (sampling mineral in the middle)
            {
                speed = gyroBot.TURN_SPEED;
                angle = -90.0 + 5.0;                  //CHECK: -90.0 in theory
                gyroBot.gyroTurn(speed, angle);
           }

            // d. drive to the starting point
            {
                //Calculate or measure the distance between landing pos and the starting point
                speed = gyroBot.DRIVE_SPEED;
                angle = angle; 
                distance = 12.0;                      //CHECK
                gyroBot.gyroDrive(speed, distance, angle);
            }

            // e. face sampling mineral in the middle
            if (false)
            {
                speed    = 0.5*gyroBot.TURN_SPEED;
                angle    = -90.0;                    //CHECK: -90.0 in theory
                gyroBot.gyroTurn(speed, angle);
            }

            // RESET angle for easier calcualtion later
            gyroBot.gyroResetAngle();

        }

        // 1. do sampling
        if (bDoSampling) {
            // a. rotate to face the gold sampling mineral
            {
                speed = gyroBot.TURN_SPEED;
                if (goldSamplePosition == tfod.GOLD_AT_LEFT) {
                    angle = 45.0 - 0.0;                  //CHECK: CCW45 in theory
                } else if (goldSamplePosition == tfod.GOLD_AT_RIGHT) {
                    angle = -45.0 + 5.0;                 //CHECK: CW 45 in theory
                } else {
                    angle = 0.0;
                }
                
                // for the CENTER case, the angle does not change from previous step, so skip it.
                if (goldSamplePosition != tfod.GOLD_AT_CENTER)
                    gyroBot.gyroTurn(speed, angle);
            }

            // b. drive to the gold sampling mineral
            {
                speed    = gyroBot.DRIVE_SPEED;
                angle    = angle; //angle does not change from previous step
                if (goldSamplePosition == tfod.GOLD_AT_LEFT) {
                    distance = 2.0 * 12.0 - 8.0;          //CHECK: 2 feet in theory
                } else if (goldSamplePosition == tfod.GOLD_AT_RIGHT) {
                    distance = 2.0 * 12.0 - 8.0;          //CHECK: 2 feet in theory
                } else { // Center
                    distance = Math.sqrt(2.0)*12.0 - 4.0; //CHECK: sqrt(2) feet in theory
                }
                gyroBot.gyroDrive(speed, distance, angle);
            }
        }

        // 2. do claiming
        if(bDoClaiming)
        {
            // a. drive back to the starting point
            {
                speed    = gyroBot.DRIVE_SPEED;
                angle    = angle; //angle does not change from previous step
                if (goldSamplePosition == tfod.GOLD_AT_LEFT) {
                    distance = -2.0 * 12.0 + 8.0;            //CHECK: 2 feet in theory
                } else if (goldSamplePosition == tfod.GOLD_AT_RIGHT) {
                    distance = -2.0 * 12.0 + 16.0;           //CHECK: 2 feet in theory
                } else { // Center
                    distance = -Math.sqrt(2.0) * 12.0 + 8.0; //CHECK: sqrt(2) feet in theory
                }
                gyroBot.gyroDrive(speed, distance, angle);
            }

            //  b. turn right to face the wall between our crater and opponent's depot (45 degree)
            {
                speed = gyroBot.TURN_SPEED;
                if (goldSamplePosition == tfod.GOLD_AT_LEFT) {
                    angle = -90.0 - 15.0;                    //CHECK: -90 in theory
                } else if (goldSamplePosition == tfod.GOLD_AT_RIGHT) {
                    angle = -90.0 - 10.0;                    //CHECK: -90 in theory
                } else { // Center
                    angle = -90.0 - 0.0;                     //CHECK: -90 in theory
                }
                gyroBot.gyroTurn(speed, angle);
            }

            // c. drive backward to the last tile next to the wall
            {
                speed    = 1.0;            //CHECK: maximum to reduce the time
                angle    = angle;          //CHECK: angle does not change
                if(goldSamplePosition == tfod.GOLD_AT_LEFT) {
                    distance = -3.0*Math.sqrt(2.0)*12.0 - 4.0;  //CHECK
                } else if(goldSamplePosition == tfod.GOLD_AT_RIGHT) {
                    distance = -3.0*Math.sqrt(2.0)*12.0 -12.0;  //CHECK
                } else {
                    distance = -3.0*Math.sqrt(2.0)*12.0 -12.0;  //CHECK
                }
                gyroBot.gyroDrive(speed, distance, angle);
            }

            // d. turn left to face our crater to make robot parallel to the wall
            {
                speed = gyroBot.TURN_SPEED;
                angle = -45.0 - 0.0 ;                //CHECK: -45.0 in theory
                gyroBot.gyroTurn(speed, angle);
            }

            // e. drive straight back to depot
            {
                speed    = 1.0;                      //CHECK: maximum to reduce the time
                angle    = angle;                    //CHECK: angle does not change
                if(goldSamplePosition == tfod.GOLD_AT_LEFT) {
                    distance = -4.0*12.0-6.0;
                } else if (goldSamplePosition == tfod.GOLD_AT_RIGHT) {
                    distance = -4.0*12.0 - 6.0;      //CHECK: 4 feet in theory
                } else {
                    distance = -4.0*12.0 + 6.0;      //CHECK: 4 feet in theory
                }
                gyroBot.gyroDrive(speed, distance, angle);
            }

            // f. drop team marker (open the claw, wait for a while and then close it)
            {
                gyroBot.robot.openGrbbers();
                sleep(500);
            }
        }

        // 3. do parking
        if (doParking)
        {
            // a. drive straigt forward to crater
            speed    = 1.0;                          //CHECK: maximum to reduce the time
            angle    = -45.0;                        //CHECK
            distance = 6.0 * 12.0 - 0.0;             //CHECK: 6 feet in theory

            gyroBot.gyroDrive(speed, distance, angle);

            // b. put the arm into crater
            {
                // Rotate the arm to make sure it just passes the 90 degree
                gyroBot.robot.swingArm.setPower(0.5);  //CHECK
                sleep(1500);                           //CHECK

                // Stop the arm
                gyroBot.robot.swingArm.setPower(0);
            }
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        //sleep(2000);
    }
}
