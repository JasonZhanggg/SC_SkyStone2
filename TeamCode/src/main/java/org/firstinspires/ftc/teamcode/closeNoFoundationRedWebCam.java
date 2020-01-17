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

    @Override
    public void runOpMode() {
        ElapsedTime time = new ElapsedTime();
        robot.init(hardwareMap, this, "autonomous");
        detector.init(hardwareMap, this);
        //wait for start
        waitForStart();
        //flip the arm down
        robot.arm.setPower(-0.4);
        sleep(600);
        robot.arm.setPower(0);
        //open the claws
        robot.claw1.setPosition(0.7);
        robot.claw2.setPosition(0.3);
        sleep(700);
        //get the skystone's position
        int blocknum = detector.getSkystone();
        telemetry.addData("Block: ", blocknum);
        telemetry.update();

        if (blocknum == 1) {
            robot.encoderDriveStrafe(0.5, 9, "left", 1.5);
        } else if (blocknum == 2) {
            robot.encoderDriveStrafe(0.5, 3, "left", 0.7);

        } else {
            robot.encoderDriveStrafe(0.5, 7, "right", 0.7);
        }
        robot.encoderDrive(0.4,0.4  , -28, -28, 2.2);
        robot.claw2.setPosition(0.6);
        sleep(600);
        robot.arm.setPower(0.4);
        sleep(250);
        robot.arm.setPower(0.15);
        //back up
        robot.encoderDrive(0.5, 0.4,9, 9, 12);
        //turn 90 degrees
        robot.encoderDrive(0.5, 0.4,-23, 23, 10);
        //drive to the foundation
        robot.driveToLine(false);
        robot.encoderDrive(0.5, 0.5,-40, -40, 10);
        //turn 90 degrees
        robot.encoderDrive(0.5, 0.4,22, -22, 10);
        //open the foundation claw and drive into the foundation
        robot.leftFoundationClaw.setPosition(0.8);
        robot.rightFoundationClaw.setPosition(0.95);
        robot.encoderDrive(0.3, 0.3,-15, -15, 10);
        //close the foundation claw
        robot.leftFoundationClaw.setPosition(0.95);
        robot.rightFoundationClaw.setPosition(0.8);
        //flip the arm down
        sleep(300);
        robot.arm.setPower(-0.4);
        sleep(200);
        robot.arm.setPower(0);
        sleep(300);
        //open the claw
        robot.claw2.setPosition(0.3);
        //flip claw back up
        while (robot.touchSensor.getState()) {
            robot.arm.setPower(0.4);
        }
        robot.arm.setPower(0);
        //move back
        robot.encoderDrive(0.4, 0.4,5, 5, 0.5);
        //turn a little so the foundation does not hit the wall
        robot.encoderDriveStrafe45(0.4, 30, "q3", 5);
        robot.encoderDrive(0.4, 0.4,-25, 25, 5); // clock wise turn

        robot.leftFoundationClaw.setPosition(0.8);
        robot.rightFoundationClaw.setPosition(0.95);
        sleep(600);
        //drive into the foundation to line it with the wall and to make the robot straight
        robot.encoderDrive(0.4, 0.4,-20, -20, 2);
        //back up onto the line
        robot.encoderDrive(0.4, 0.4,45, 45, 4);
    }

}
