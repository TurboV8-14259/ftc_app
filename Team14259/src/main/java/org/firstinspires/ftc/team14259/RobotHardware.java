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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_wheel"
 * Motor channel:  Right drive motor:        "right_wheel"
 * Motor channel:  Right lift motor:         "right_lift"
 * Motor channel:  Arm swing motor:          "arm_swing"
 * Motor channel:  Arm extension motor:      "arm_extension"
 * Servo channel:  Servo to open left claw:  "left_grabber"
 * Servo channel:  Servo to open right claw: "right_grabber"
 *
 */
public class RobotHardware
{
    /* Public OpMode members. */
    public DcMotor  leftDrive    = null;
    public DcMotor  rightDrive   = null;
    public DcMotor  rightLift    = null;
    public DcMotor  swingArm     = null;
    public DcMotor  extensionArm = null;
    public Servo    leftGrabber  = null;
    public Servo    rightGrabber = null;

    public static final double ROBOT_L_R_SIZE   = 14.5;
    public static final double ROBOT_F_B_SIZE   = 16.0;
    public static final double ROBOT_HEIGHT     = 18.0;
    public static final double ROBOT_CENTER_L_R = 14.5/2.0;
    public static final double ROBOT_CENTER_F_B = 16.0/2.0; //CHECK

    public static final double LEFT_GRABBER_CLOSE_POS  = 1.0;
    public static final double RIGHT_GRABBER_CLOSE_POS = 0.0;
    public static final double LEFT_GRABBER_OPEN_POS   = 0.0;
    public static final double RIGHT_GRABBER_OPEN_POS  = 1.0;

    /* local OpMode members. */
    HardwareMap hwMap =  null;

    /* Constructor */
    public RobotHardware(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive    = hwMap.get(DcMotor.class, "left_wheel");
        rightDrive   = hwMap.get(DcMotor.class, "right_wheel");
        rightLift    = hwMap.get(DcMotor.class, "right_lift");
        swingArm     = hwMap.get(DcMotor.class, "arm_swing");
        extensionArm = hwMap.get(DcMotor.class, "arm_extension");

        // Set direction for all motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        swingArm.setDirection(DcMotor.Direction.REVERSE);
        extensionArm.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        rightLift.setPower(0);
        swingArm.setPower(0);
        extensionArm.setPower(0);

        // Set all motors to run with/without encoders.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        swingArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        leftGrabber  = hwMap.get(Servo.class, "left_grabber");
        rightGrabber = hwMap.get(Servo.class, "right_grabber");
        leftGrabber.setPosition(LEFT_GRABBER_CLOSE_POS);
        rightGrabber.setPosition(RIGHT_GRABBER_CLOSE_POS);
    }

    /*  Open grabbber interface */
    public void openGrbbers() {
        // Open grabber
        leftGrabber.setPosition(LEFT_GRABBER_OPEN_POS);
        rightGrabber.setPosition(RIGHT_GRABBER_OPEN_POS);
    }

    /*  Close grabbber interface */
    public void closeGrbbers() {
        // Close grabber
        leftGrabber.setPosition(LEFT_GRABBER_CLOSE_POS);
        rightGrabber.setPosition(RIGHT_GRABBER_CLOSE_POS);
    }
}

