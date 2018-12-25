/* Copyright (c) 2018 FIRST. All rights reserved.
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

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class TensorFlowObjectDetect {
    private static final String TFOD_MODEL_ASSET     = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL   = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // Use Webcam or phone back camera to find out the position for the sampling gold mineral
    private static final boolean bUseWebcam = true;

    // Targeted confidence level for TFOD
    public static final double TFOD_MINIMUM_CONFIDENCE_LEVEL  = 0.50; // 50 percent

    // Gold mineral positions
    public static final int GOLD_AT_LEFT   = -1;
    public static final int GOLD_AT_CENTER = 0;
    public static final int GOLD_AT_RIGHT  = 1;

    // Local hardwareMap to get a handle to camera
    HardwareMap   hwMap = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    // Callback to Depot or Crater to check if opModeIsActive
    private OnOpEventListener mListener;

    // Set the listner
    public void registerOnOpEventListener(OnOpEventListener listener) {
        this.mListener = listener;
    }

    // Local function to do callbacks
    public boolean opModeIsActive() {
        if (this.mListener != null)
            return mListener.onOpIsActiveEvent();
        else 
            return true;
    }

    public void telemetryAddData(String sDataItem, String sData) {
        if (this.mListener != null)
            mListener.onTelemetryAddData(sDataItem, sData);
    }

    public void telemetryUpdate() {
        if (this.mListener != null)
            mListener.onTelemetryUpdate();
    }

    /* Constructor */
    public TensorFlowObjectDetect(){
    }

    /* TFOD Initialization */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // TFOD uses the camera frames from the VuforiaLocalizer, so we create that first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetryAddData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    /* TFOD detection */
    public int detect(double seconds) {
        int retGoldSamplePosition = GOLD_AT_CENTER;
        ElapsedTime detectTimer = new ElapsedTime();

        // Reset timer
        detectTimer.reset();

        // Activate Tensor Flow Object Detection
        if ((tfod != null) && opModeIsActive()) {
                tfod.activate();
        }

        while ((tfod != null) && opModeIsActive() && (detectTimer.time() < seconds)) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int size = updatedRecognitions.size();
                telemetryAddData("# Object Detected", ""+size);
                if (size == 3) { // Assume camera is setup to be able to find 3 minerals
                    int goldMineralX    = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                           silverMineral2X = (int) recognition.getLeft();
                        }
                    }

                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            retGoldSamplePosition = GOLD_AT_LEFT;
                            telemetryAddData("Gold Mineral Position", "Left");
                            telemetryUpdate();
                            break;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            retGoldSamplePosition = GOLD_AT_RIGHT;
                            telemetryAddData("Gold Mineral Position", "Right");
                            telemetryUpdate();
                            break;
                        } else {
                            retGoldSamplePosition = GOLD_AT_CENTER;
                            telemetryAddData("Gold Mineral Position", "Center");
                            telemetryUpdate();
                            break;
                        }
                    } 
                }

/*
                if (size == 2) { // Assume camera cannot see the right one no matter what
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                           silverMineral2X = (int) recognition.getLeft();
                        }
                    }

                    if (goldMineralX == -1) {
                        retGoldSamplePosition = GOLD_AT_RIGHT;
                        telemetryAddData("Gold Mineral Position", "Right");
                        telemetryUpdate();
                        break;
                    } else { //one silver has to be -1 since gold is not and only 2 detected
                        int silverToBeCompared = silverMineral1X;
                        if (silverMineral2X != -1) {
                            silverToBeCompared = silverMineral2X;
                        }
                        if (goldMineralX < silverToBeCompared) {
                            retGoldSamplePosition = GOLD_AT_LEFT;
                            telemetryAddData("Gold Mineral Position", "Left");
                            telemetryUpdate();
                            break;
                        } else {
                            retGoldSamplePosition = GOLD_AT_CENTER;
                            telemetryAddData("Gold Mineral Position", "Left");
                            telemetryUpdate();
                            break;
                        }
                    }
                }
*/
                telemetryUpdate();
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        return retGoldSamplePosition;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VuforiaKey.VUFORIA_KEY; //VUFORIA_KEY;

        // Choose one of the following depending on which camera is used
        if (bUseWebcam)
            parameters.cameraName = hwMap.get(WebcamName.class, "Webcam"); //Webcam
        else
            parameters.cameraDirection = CameraDirection.BACK; //Phone back camera

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = TFOD_MINIMUM_CONFIDENCE_LEVEL;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
