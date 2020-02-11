
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "blueSide", group = "Concept")
//@Disabled
public class closeNoFoundationBlueWebCam extends LinearOpMode {

    SkystoneDetector detector = new SkystoneDetector();
    SkystoneHardware robot = new SkystoneHardware();
    int extra = 0;
    @Override
    public void runOpMode() {
        //init the hardware map and the skystone detection
        robot.init(hardwareMap, this, "autonomous");
        detector.init(hardwareMap, this);

        //wait for start
        waitForStart();
        //flip the arm down
        robot.arm.setPower(-0.4);
        sleep(600);
        robot.arm.setPower(0);
        //open the claws
        robot.claw1.setPosition(0.78);
        robot.claw2.setPosition(0.19);
        sleep(700);
        //get the skystone's position
        int blocknum = detector.getSkystone();
        telemetry.addData("Block: ", blocknum);
        telemetry.update();

        //move to the skystone
        if (blocknum == 1) {
            robot.encoderDriveStrafe(0.6, 12, "left", 1.5);
        } else if (blocknum == 2) {
            robot.encoderDriveStrafe(0.6, 7.5, "left", 0.7);
            extra = 8;

        } else {
            robot.encoderDriveStrafe(0.6, 2.5, "right", 0.7);
            extra = 16;
        }


        robot.encoderDrive(0.4, 0.4, -25, -25, 10);
        //drive forward to the skystone, pick it up, raise the arm partially, and lock the arm
        //robot.encoderDrive(0.4, 0.4,-28, -28, 2.2);
        robot.claw1.setPosition(0.45);
        sleep(600);
        robot.arm.setPower(0.4);
        sleep(250);
        robot.arm.setPower(0.12);
        //back up
        robot.encoderDrive(0.6, 0.6,9, 9, 12);

        robot.encoderDriveStrafe(0.8, 35+extra,"left",  10);
        robot.encoderDrive(0.6, 0.6,-5, -5, 10);
        //close the foundation claw

        //flip the arm down
        sleep(300);
        robot.arm.setPower(-0.4);
        sleep(200);
        robot.arm.setPower(0);
        sleep(300);
        //open the claw
        robot.claw1.setPosition(0.7);
        //flip claw back up
        robot.arm.setPower(0.4);
        sleep(600);
        robot.arm.setPower(0);
        robot.encoderDrive(0.6, 0.6,15, 15, 10);
        robot.encoderDriveStrafe(0.8, 60+extra,"right",  10);

        robot.arm.setPower(-0.4);
        sleep(600);
        robot.arm.setPower(0);
        //open the claws
        robot.claw1.setPosition(0.78);
        robot.claw2.setPosition(0.19);
        robot.encoderDrive(0.6, 0.6,-10, -10, 10);

        robot.claw1.setPosition(0.45);
        sleep(600);
        robot.arm.setPower(0.4);
        sleep(250);
        robot.arm.setPower(0.12);
        //back up
        robot.encoderDrive(0.6, 0.6,9, 9, 12);
        robot.encoderDriveStrafe(0.8, 70+extra,"left",  10);
        //open the foundation claw and drive into the foundation
        robot.leftFoundationClaw.setPosition(0.8);
        robot.rightFoundationClaw.setPosition(0.95);
        robot.encoderDrive(0.6, 0.6,-15, -15, 10);
        robot.leftFoundationClaw.setPosition(0.95);
        robot.rightFoundationClaw.setPosition(0.8);
        //move back
        robot.encoderDrive(0.6, 0.6,5, 5, 0.5);
        //turn a little so the foundation does not hit the wall
        robot.encoderDriveStrafe45(0.4, 30, "q4", 5);
        robot.encoderDrive(0.6, 0.6,25, -25, 5); // clock wise turn

        robot.leftFoundationClaw.setPosition(0.8);
        robot.rightFoundationClaw.setPosition(0.95);
        sleep(600);
        //drive into the foundation to line it with the wall and to make the robot straight
        robot.encoderDrive(0.6, 0.6,-20, -20, 2);
        //back up onto the line
        robot.encoderDrive(0.8, 0.8,45, 45, 4);
    }
}
