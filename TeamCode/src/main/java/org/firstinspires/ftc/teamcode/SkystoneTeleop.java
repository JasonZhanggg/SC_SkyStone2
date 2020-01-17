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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "MecDrive", group = "Pushbot")
//@Disabled
public class SkystoneTeleop extends LinearOpMode {

    SkystoneHardware robot = new SkystoneHardware();
    double ch3 = 0;
    double ch1;
    double ch4 = 0;
    double speed = 0;
    double spinValue = 0;
    ElapsedTime flipTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this, "teleop");
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
        boolean flag = true; //lift flag
        boolean armFlag = true; //arm flag
        boolean timerFlag = true;
        boolean flipArmFlag = true; //flip arm flag
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                speed = 0.3;
            } else if (gamepad1.a) {
                speed = 0.6;
            } else if (gamepad1.dpad_down) {
                speed = 0.15;
            }
            //close
            if (gamepad2.x) {
                robot.claw1.setPosition(0.35);
                robot.claw2.setPosition(0.6);
            }
            //open
            if (gamepad2.y) {
                robot.claw1.setPosition(0.7);
                robot.claw2.setPosition(0.3);
            }
            if (gamepad2.right_stick_y > 0.6 && armFlag) {
                robot.arm.setPower(-0.4);
                flipArmFlag = true;
            } else if (gamepad2.right_stick_y < -0.6 && armFlag) {
                robot.arm.setPower(0.4);
                flipArmFlag = true;
            } else if (armFlag && flipArmFlag) {
                robot.arm.setPower(0);
            }

            if (gamepad2.left_stick_x > 0.8) {
                flag = false;
                robot.lift.setPower(-0.1);

            }

            if (gamepad2.left_stick_y > 0.6 && robot.touchSensorLift.getState()) {
                flag = true;
                robot.lift.setPower(0.2);
            } else if (gamepad2.left_stick_y < -0.6) {
                flag = true;
                robot.lift.setPower(-0.55);
            } else if (flag) {
                robot.lift.setPower(0);
            }
            if (gamepad2.left_trigger > 0.4) {
                spinValue = 0;

            } else if (gamepad2.right_trigger > 0.4 && !robot.touchFlipper2.isPressed()) {
                spinValue = 1;
            } else {
                spinValue = 0.5;
            }
            //open
            if (gamepad2.a) {
                robot.leftClamp.setPosition(0);
                robot.rightClamp.setPosition(0.2);

            }
            //close
            else if (gamepad2.b) {
                robot.leftClamp.setPosition(0.13);
                robot.rightClamp.setPosition(0.07);

            }
            if (!robot.touchSensor.getState()) {
                robot.arm.setPower(0);
                robot.claw1.setPosition(0.55);
                robot.claw2.setPosition(0.45);
                armFlag = true;

                if (flipTimer.time() >= 0.3 && timerFlag) {
                    robot.leftClamp.setPosition(0.13);
                    robot.rightClamp.setPosition(0.07);
                    timerFlag = false;
                }
                else if(timerFlag){
                    flipTimer.reset();
                    flipTimer.startTime();
                }
            }
            else {
                timerFlag = true;
            }
            robot.rotation_servo.setPosition(spinValue);
            if (gamepad2.left_bumper) {
                robot.leftClamp.setPosition(0);
                robot.rightClamp.setPosition(0.2);
                robot.arm.setPower(0.4);
                armFlag = false;
            }
            if (gamepad1.y) {
                robot.rightFoundationClaw.setPosition(0.95);
                robot.leftFoundationClaw.setPosition(0.8);
            } else if (gamepad1.x) {
                robot.rightFoundationClaw.setPosition(0.8);
                robot.leftFoundationClaw.setPosition(0.95);
            }
            if (gamepad2.right_stick_x > 0.8) {
                robot.arm.setPower(0.1);
                flipArmFlag = false;
            }


            ch3 = gamepad1.left_stick_y > 0.4 ? -0.9 : gamepad1.left_stick_y < -0.4 ? 0.9 : 0;
            ch4 = gamepad1.right_bumper ? 0.9 : gamepad1.left_bumper ? -0.9 : 0;
            ch1 = gamepad1.right_stick_x > 0.4 ? 0.6 : gamepad1.right_stick_x < -0.4 ? -0.6 : 0;
            robot.leftDriveFront.setPower((ch3 + ch1 + ch4) * speed);
            robot.leftDriveBack.setPower((ch3 + ch1 - ch4) * speed);
            robot.rightDriveFront.setPower((ch3 - ch1 - ch4) * speed);
            robot.rightDriveBack.setPower((ch3 - ch1 + ch4) * speed);

        }
    }
}
