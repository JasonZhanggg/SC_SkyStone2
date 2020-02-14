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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "MecDrive3", group = "Pushbot")
//@Disabled
public class SkystoneTeleop3 extends LinearOpMode {

    SkystoneHardware robot = new SkystoneHardware();
    double ch3 = 0;
    double ch1;
    double ch4 = 0;
    double speed = 0;
    ElapsedTime flipTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this, "teleop");
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
        boolean flag = true; //lift flag
        int timerFlag = 1;
        boolean flipArmFlag = true; //flip arm flag
        boolean rangeFlag = false;
        double l = 0, r = 0; // front range sensors reading

        waitForStart();

        while (opModeIsActive()) {
            // ### Control speed
            if (gamepad1.b) {
                speed = 0.3;
            } else if (gamepad1.a) {
                speed = 0.6;
            } else if (gamepad1.dpad_down) {
                speed = 0.15;
            }

            // ### Open/close the intake claw
            if (gamepad2.x) {
                robot.closeIntakeClaw(3);
            }
            if (gamepad2.y) {
                robot.openIntakeClaw();
            }

            // ###  Control (flip) the intake arm
            if (gamepad2.right_stick_y > 0.6) {
                robot.arm.setPower(-0.4);
                timerFlag = 1;  // reset timerFlag so we can start it over again
                flipArmFlag = true;
            } else if (gamepad2.right_stick_y < -0.6) {
                robot.arm.setPower(0.4);
                flipArmFlag = true;
            } else if (flipArmFlag) {
                robot.arm.setPower(0);
            }

            // ### Control (move up and down) lift slider

            if (gamepad2.left_stick_y > 0.6 && robot.touchSensorLift.getState()) {
                robot.lift.setPower(0.2);
            }
            else if (gamepad2.left_stick_y < -0.6) {
                robot.lift.setPower(-0.55);
            }
            else if (!robot.touchSensorLift.getState()) {
                robot.lift.setPower(0);
            }
            else {
                robot.lift.setPower(-0.1);
            }
            // ### Control (rotate) the back Flipper Motor
            // Game pad 2 left trigger flippers backward (place the stone on the foundation), use more power
            // Game pad 2 right trigger flippers forward (return to the position to pick up the stone), use less power
            // touchFlipper1 (the front touch sensor)
            // touchFlipper2 (the back touch sensor)
            if (gamepad2.right_trigger > 0.4) {
                if (!robot.touchFlipper2.getState()) {
                    robot.flipperMotor.setPower(0);
                } else {
                    robot.flipperMotor.setPower(0.45);
                }
            } else if (gamepad2.left_trigger > 0.4) {
                if (!robot.touchFlipper1.getState()) {
                    robot.flipperMotor.setPower(0);
                } else {
                    robot.flipperMotor.setPower(-0.3);
                }
            } else {
                robot.flipperMotor.setPower(0);
            }

            // ### Open/close the flipper claw (back)
            if (gamepad2.a) {
                robot.openFlipperClaw();
            } else if (gamepad2.b) {
                robot.closeFlipperClaw();
            }

            // ### Intake touch sensor (front) pressed, let's do some magics to make it easy for the driver
            // timerFlag:
            //  1 = init state
            //  2 = intake arm flipped
            //  3 = claw closed
            if (!robot.touchSensor.getState() && timerFlag == 1) {
                robot.arm.setPower(0);
                robot.releaseIntakeClaw();
                flipTimer.reset();
                flipTimer.startTime();
                timerFlag = 2;
            }
            if (flipTimer.time() > 0.2 && flipTimer.time() < 0.3 && timerFlag == 2) {
                robot.closeFlipperClaw();
                timerFlag = 3;
            }

            // Hold the arm
            if (gamepad2.right_stick_x > 0.8) {
                robot.arm.setPower(0.1);
                flipArmFlag = false;
            }

            // ### Use range sensors to scan the stone and auto grab it
            // Game pad 1 right trigger to enable auto scan and left trigger to disable it
            if (rangeFlag) {
                l = robot.rangeSensorL.getDistance(DistanceUnit.CM);
                r = robot.rangeSensorR.getDistance(DistanceUnit.CM);
                telemetry.addData("L cm", (int) l);
                telemetry.addData("R cm", (int) r);
            }
            if (gamepad1.left_trigger > 0.6) {
                rangeFlag = false;
            } else if (gamepad1.right_trigger > 0.6) {
                rangeFlag = true;
            }
            if (l > 5.0 && l < 9.0 && r > 5.0 && r < 9.0 && rangeFlag) {
                telemetry.addData("Status:", "Locked");
                robot.closeIntakeClaw(3);
            } else {
                telemetry.addData("Status:", "Free");
            }
            telemetry.addData("Scan:", rangeFlag);
            telemetry.update();

            // ### Drive the robot
            ch3 = gamepad1.left_stick_y > 0.4 ? 0.9 : gamepad1.left_stick_y < -0.4 ? -0.9 : 0;
            ch4 = gamepad1.right_bumper ? -0.9 : gamepad1.left_bumper ? 0.9 : 0;
            ch1 = gamepad1.right_stick_x > 0.4 ? -0.6 : gamepad1.right_stick_x < -0.4 ? 0.6 : 0;
            robot.leftDriveFront.setPower((ch3 + ch1 + ch4) * speed);
            robot.leftDriveBack.setPower((ch3 + ch1 - ch4) * speed);
            robot.rightDriveFront.setPower((ch3 - ch1 - ch4) * speed);
            robot.rightDriveBack.setPower((ch3 - ch1 + ch4) * speed);

            // ### Control foundation claws
            //if (gamepad1.y) {
            //   robot.frontFoundationClawUp();
            //} else if (gamepad1.x) {
            //     robot.frontFoundationClawDown();
            // }
            if (gamepad1.x) {
                robot.backFoundationClawDown();
            } else if (gamepad1.y) {
                robot.backFoundationClawUp();
            }
        }
    }
}
