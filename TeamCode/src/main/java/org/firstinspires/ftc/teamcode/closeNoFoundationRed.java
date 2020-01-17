/* Copyright (c) 2019 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "ClOsEnOfOuNdAtIoNrEd", group = "Concept")
//@Disabled
public class closeNoFoundationRed extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String Stone = "Stone";
    private static final String Skystone = "Skystone";


    private static final String VUFORIA_KEY =
            "AXx1coP/////AAAAGR5+EbHI40Yxm9qZuu+1B24qD4W8rX1Mpr3bUfT5zyb1j8gA4EQSLVMVxsL42LqB8fa6x/dQxnRpoAYen7mztKaCwY706bIMXjW/1n7YZHJ+/v4ZlzL39Bk5NHHMOvWrZ2nzRz2aE3ZM7dfUQFyOr0hIl5/cfmnSQ+MWUDJWynCWpinkTc8/CaHLh9f7O+jLy1zPzrS9nDJSj7GUXDRGS5+Aeh/Wwv0wvmfdz8ZdPA1zA0iXF1KqvfkjmMv25F15ikWX7/+XuwXlvHIEw1vruo9zbCaQcmgqve05tldl1y9aiMcdVeEJUrQLhdYA2ct8aKpqRIs6MXaw4otzbED5Zn93a4Wn0kcLBRUOQ0OJ6li0";


    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    OldSkystoneHardware robot = new OldSkystoneHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this, "autonomous");
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        robot.encoderDrive(0.23, -20, -20, 12);
        robot.encoderDriveStrafe(0.2,4,"left");
        int skystoneNumber = 0;
        outerloop:
        while (opModeIsActive()) {


            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    // robot.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //robot.leftDriveBack.setPower(0.19);
                    //robot.leftDriveFront.setPower(-0.19);
                    //robot.rightDriveBack.setPower(-0.19);
                    // robot.rightDriveFront.setPower(0.19);
                    robot.encoderDriveStrafe(0.19, 8, "left");
                    sleep(1000);
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals("Skystone")) {
                            telemetry.addData("I", "found a skystone");
                            telemetry.update();
                            robot.setDrivePower(0);
                            break outerloop;
                        }
                    }
                    skystoneNumber++;
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }

        robot.encoderDriveStrafe(0.2, 10, "right");
        /**
         * PICK UP SKYSTONE HERE
         */
        telemetry.addData("Skystone was at position", skystoneNumber);
        telemetry.update();
        robot.encoderDrive(0.2, 5, 5, 12);
        robot.encoderDriveStrafe(0.2, 20, "right");
        robot.encoderDrive(0.2, -15, 15, 12);
        /**
         * DROP SKYSTONE HERE
         */
        robot.encoderDriveStrafe(0.2, 5, "right");
        robot.encoderDrive(0.2, -10, -10, 12);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, Stone, Skystone);
    }
}
