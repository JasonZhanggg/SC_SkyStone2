
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous(name = "blueSide", group = "Concept")
//@Disabled
public class closeNoFoundationBlueWebCam extends LinearOpMode {

    SkystoneDetector detector = new SkystoneDetector();
    SkystoneHardware robot = new SkystoneHardware();
    int extra = 0;
    int pos = 3; //use both claws

    @Override
    public void runOpMode() {
        //init the hardware map and the skystone detection
        robot.init(hardwareMap, this, "autonomous");
        detector.init(hardwareMap, this);
        //wait for start
        waitForStart();

        int blockNum = detector.getSkystone();

        //move to the skystone
            if (blockNum == 1) {
            robot.encoderDriveStrafe(0.5, 11, "left", 1.5);
        } else if (blockNum == 2) {
            robot.encoderDriveStrafe(0.5, 2, "left", 0.7);
            extra = 8;
        } else {
            robot.encoderDriveStrafe(0.5, 7, "right", 0.7);
            extra = 16;
    }
        robot.grabStone(pos, 28, 8);
        robot.encoderDriveStrafe(0.7, 65+extra,"left",  10);
        robot.dropStone(0, 3);
        robot.encoderDriveStrafe(0.7, 85+extra,"right",  10);
        robot.grabStone(pos, 8, 6);
        robot.encoderDriveStrafe(0.7, 100+extra,"left",  10);
        robot.dropStone(8, 0);
        robot.moveFoundation("blue");


    }
}
