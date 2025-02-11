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


@Autonomous(name = "foundationRed", group = "Concept")
//@Disabled
public class foundationRed extends LinearOpMode {

    SkystoneHardware robot = new SkystoneHardware();

    @Override
    public void runOpMode() {
        waitForStart();
        robot.init(hardwareMap, this, "autonomous");

        // Test BLUE side foundation move
       // robot.encoderDriveStrafe45(0.4, 30, "q4", 5);
       // robot.encoderDrive(0.4, 0.4,25, -25, 5);  // counter clock wise turn
        robot.leftFoundationClaw.setPosition(0.8);
        robot.rightFoundationClaw.setPosition(0.95);
        // Test RED side foundation move
        robot.encoderDriveStrafe(0.4, 12,"right", 0.5);
        robot.encoderDrive(0.3, 0.3,-35, - 35, 5);
        robot.leftFoundationClaw.setPosition(0.95);
        robot.rightFoundationClaw.setPosition(0.8);
        sleep(1000);
        robot.encoderDrive(0.4, 0.4,5, 5, 0.5);

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
