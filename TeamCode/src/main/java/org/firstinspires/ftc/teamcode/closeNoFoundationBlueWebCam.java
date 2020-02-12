
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "blueSide", group = "Concept")
//@Disabled
public class closeNoFoundationBlueWebCam extends LinearOpMode {

    SkystoneDetector detector = new SkystoneDetector();
    SkystoneHardware robot = new SkystoneHardware();
    int extra = 0;
    int pos = 1; //use the right claw
    @Override
    public void runOpMode() {
        //init the hardware map and the skystone detection
        robot.init(hardwareMap, this, "autonomous");
        detector.init(hardwareMap, this);
        //wait for start
        waitForStart();
        robot.openIntakeClaw();
        int blockNum = detector.getSkystone();

        //move to the skystone
        if (blockNum == 1) {
            robot.encoderDriveStrafe(0.6, 15, "left", 1.5);
        } else if (blockNum == 2) {
            robot.encoderDriveStrafe(0.6, 8, "left", 0.7);
            extra = 8;

        } else {
            robot.encoderDriveStrafe(0.6, 3, "right", 0.7);
            extra = 16;
        }

        robot.grabStone(pos, 26, 8);
        robot.encoderDriveStrafe(0.8, 65+extra,"left",  10);
        robot.dropStone(8, 8);
        robot.encoderDriveStrafe(0.8, 90+extra,"right",  10);
        robot.grabStone(pos, 12, 9);
        robot.encoderDriveStrafe(0.8, 100+extra,"left",  10);
        robot.dropStone(8, 0);
        robot.moveFoundation("blue");
    }
}
