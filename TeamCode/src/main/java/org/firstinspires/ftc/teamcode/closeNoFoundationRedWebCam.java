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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "redSide", group = "Concept")
//@Disabled
public class closeNoFoundationRedWebCam extends LinearOpMode {

    SkystoneHardware robot = new SkystoneHardware();
    SkystoneDetector detector = new SkystoneDetector();

    int extra = 0;
    int pos = 1; //use the left claw
    @Override
    public void runOpMode() {
        //init the hardware map and the skystone detection
        robot.init(hardwareMap, this, "autonomous");
        detector.init(hardwareMap, this);
        //wait for start
        waitForStart();
        robot.openIntakeClaw();
        int blockNum = detector.getSkystone();

        if (blockNum == 1) {
            robot.encoderDriveStrafe(0.5, 9, "left", 1.5);
        } else if (blockNum == 2) {
            robot.encoderDriveStrafe(0.5, 3, "left", 0.7);

        } else {
            robot.encoderDriveStrafe(0.5, 7, "right", 0.7);
        }
        robot.grabStone(pos, 27, 8);
        robot.encoderDriveStrafe(0.8, 65+extra,"right",  10);
        robot.dropStone(5, 5);
        robot.encoderDriveStrafe(0.8, 90+extra,"left",  10);
        robot.grabStone(pos, 12, 9);
        robot.encoderDriveStrafe(0.8, 100+extra,"right",  10);
        robot.dropStone(5, 0);
        robot.moveFoundation("red");
    }

}
