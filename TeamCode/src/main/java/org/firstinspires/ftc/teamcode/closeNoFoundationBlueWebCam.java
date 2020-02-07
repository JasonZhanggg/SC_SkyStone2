
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "blueSide", group = "Concept")
//@Disabled
public class closeNoFoundationBlueWebCam extends LinearOpMode {

    SkystoneDetector detector = new SkystoneDetector();
    SkystoneHardware robot = new SkystoneHardware();

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
            robot.encoderDriveStrafe(0.5, 16, "left", 1.5);
        } else if (blocknum == 2) {
            robot.encoderDriveStrafe(0.4, 7.5, "left", 0.7);

        } else {
            robot.encoderDriveStrafe(0.5, 2, "right", 0.7);
        }
        robot.encoderDrive(0.4, 0.4, -18, -18, 10);
        double l = robot.rangeSensorL.getDistance(DistanceUnit.CM);
        double r = robot.rangeSensorR.getDistance(DistanceUnit.CM);

        while(l>6.0&&r>6.0){
            l = robot.rangeSensorL.getDistance(DistanceUnit.CM);
            r = robot.rangeSensorR.getDistance(DistanceUnit.CM);
            robot.setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.setDrivePower(-0.2);
        }
        robot.setDrivePower(0);
        //drive forward to the skystone, pick it up, raise the arm partially, and lock the arm
        //robot.encoderDrive(0.4, 0.4,-28, -28, 2.2);
        robot.claw1.setPosition(0.45);
        sleep(600);
        robot.arm.setPower(0.4);
        sleep(250);
        robot.arm.setPower(0.1);
        //back up
        robot.encoderDrive(0.4, 0.4,9, 9, 12);
        //turn 90 degrees
        robot.encoderDrive(0.4, 0.4,23, -23, 10);
        //drive to the foundation
        robot.driveToLine(true);
        robot.encoderDrive(0.5,0.5, -40, -40, 10);
        //turn 90 degrees
        robot.encoderDrive(0.4, 0.4,-22, 22, 10);
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
        robot.claw1.setPosition(0.7);
        //flip claw back up
        robot.arm.setPower(0.4);
        sleep(600);
        robot.arm.setPower(0);

        //move back
        robot.encoderDrive(0.4, 0.4,5, 5, 0.5);
        //turn a little so the foundation does not hit the wall
        robot.encoderDriveStrafe45(0.4, 30, "q4", 5);
        robot.encoderDrive(0.4, 0.4,25, -25, 5); // clock wise turn

        robot.leftFoundationClaw.setPosition(0.8);
        robot.rightFoundationClaw.setPosition(0.95);
        sleep(600);
        //drive into the foundation to line it with the wall and to make the robot straight
        robot.encoderDrive(0.4, 0.4,-20, -20, 2);
        //back up onto the line
        robot.encoderDrive(0.4, 0.4,45, 45, 4);
    }
}
