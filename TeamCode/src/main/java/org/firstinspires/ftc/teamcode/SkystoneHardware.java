
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class SkystoneHardware {
    /* Public OpMode members. */

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    //static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: 5202 Series Yellow Jacket Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415926535);
    LinearOpMode op_mode = null;
    public DcMotor leftDriveFront = null;
    public DcMotor rightDriveBack = null;
    public DcMotor leftDriveBack = null;
    public DcMotor rightDriveFront = null;
    public DcMotor flipperMotor = null;
    public Servo claw1 = null;
    public Servo claw2 = null;

    public Servo rightFoundationClaw = null;
    public Servo leftFoundationClaw = null;
    public Servo backFoundationClaw = null;
    public Servo leftClamp = null;
    public Servo rightClamp = null;
    public DcMotor lift = null;
    HardwareMap hwMap = null;
    public DcMotor arm = null;
    public Servo rotation_servo = null;
    public NormalizedColorSensor colorSensor = null;
    public ModernRoboticsI2cRangeSensor rangeSensorL,rangeSensorR;
    public DigitalChannel touchSensor;  // Hardware Device Object
    public DigitalChannel touchSensorLift;  // Hardware Device Object
    public DigitalChannel touchFlipper1, touchFlipper2;


    ElapsedTime runtime = new ElapsedTime();


    public SkystoneHardware() {
    }

    public void init(HardwareMap ahwMap, LinearOpMode opMode, String runeMode) {
        op_mode = opMode;
        hwMap = ahwMap;
        //arm = hwMap.get(DcMotor.class, "arm");

        leftDriveFront  = hwMap.get(DcMotor.class, "leftf");
        rightDriveFront = hwMap.get(DcMotor.class, "rightf");
        leftDriveBack   = hwMap.get(DcMotor.class, "leftb");
        rightDriveBack  = hwMap.get(DcMotor.class, "rightb");
        flipperMotor    = hwMap.get(DcMotor.class, "flipper_motor");

        arm = hwMap.get(DcMotor.class, "arm");
        lift = hwMap.get(DcMotor.class, "lift");
        claw1 = hwMap.get(Servo.class, "claw1"); // right
        claw2 = hwMap.get(Servo.class, "claw2"); // left
        rightFoundationClaw = hwMap.get(Servo.class, "rightFoundationClaw");
        leftFoundationClaw = hwMap.get(Servo.class, "leftFoundationClaw");
        backFoundationClaw = hwMap.get(Servo.class, "backFoundationClaw");

        rangeSensorL = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangel");
        rangeSensorR = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ranger");
        touchSensor = hwMap.get(DigitalChannel.class, "touch");
        touchSensorLift = hwMap.get(DigitalChannel.class, "touchLift");
        leftClamp = hwMap.get(Servo.class, "leftClamp");
        rightClamp = hwMap.get(Servo.class, "rightClamp");
        rotation_servo = hwMap.get(Servo.class, "rotation_servo");
        colorSensor = hwMap.get(NormalizedColorSensor.class, "sensor_color");

        // 1 is the front, 2 is the back
        touchFlipper1 = hwMap.get(DigitalChannel.class, "touchFlipper1");
        touchFlipper2 = hwMap.get(DigitalChannel.class, "touchFlipper2");

        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        //arm.setDirection(DcMotor.Direction.FORWARD);
        //arm.setPower(0);
        leftDriveFront.setPower(0);
        rightDriveFront.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        flipperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (runeMode.equals("teleop")) {
            setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (runeMode.equals("autonomous")) {
            setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public void setDriveRunMode(DcMotor.RunMode runmode) {
        leftDriveFront.setMode(runmode);
        leftDriveBack.setMode(runmode);
        rightDriveFront.setMode(runmode);
        rightDriveBack.setMode(runmode);
    }

    public void encoderDrive(double speedLeft, double speedRight,
                             double leftInches, double rightInches,
                             double timeoutS) {

        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newLeftTargetFront;
        int newLeftTargetBack;
        int newRightTargetBack;
        int newRightTargetFront;
        // Ensure that the opmode is still active
        if (op_mode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newRightTargetBack = rightDriveBack.getCurrentPosition() + (int) (-rightInches * COUNTS_PER_INCH);
            newRightTargetFront = rightDriveFront.getCurrentPosition() + (int) (-rightInches * COUNTS_PER_INCH);
            newLeftTargetBack = leftDriveBack.getCurrentPosition() + (int) (-leftInches * COUNTS_PER_INCH);
            newLeftTargetFront = leftDriveFront.getCurrentPosition() + (int) (-leftInches * COUNTS_PER_INCH);

            leftDriveFront.setTargetPosition(newLeftTargetFront);
            leftDriveBack.setTargetPosition(newLeftTargetBack);
            rightDriveFront.setTargetPosition(newRightTargetFront);
            rightDriveBack.setTargetPosition(newRightTargetBack);

            // Turn On RUN_TO_POSITION
            setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            runtime.startTime();
            leftDriveFront.setPower(Math.abs(speedLeft));
            leftDriveBack.setPower(Math.abs(speedLeft));
            rightDriveFront.setPower(Math.abs(speedRight));
            rightDriveBack.setPower(Math.abs(speedRight));
            while (op_mode.opModeIsActive() &&
                    (leftDriveBack.isBusy() && rightDriveBack.isBusy() &&
                            leftDriveFront.isBusy() && rightDriveFront.isBusy()) && runtime.time()<timeoutS) {

                // Display it for the driver.
                op_mode.telemetry.addData("Path_1", "Running to %7d :%7d: %7d :%7d",
                        newLeftTargetFront, newRightTargetFront, newLeftTargetBack, newRightTargetBack);
                op_mode.telemetry.addData("Path_2", "Running at %7d :%7d: %7d : %7d",
                        leftDriveFront.getCurrentPosition(),
                        rightDriveFront.getCurrentPosition(),
                        leftDriveBack.getCurrentPosition(),
                        rightDriveBack.getCurrentPosition());
                op_mode.telemetry.addData("Time: ", runtime.time());
                op_mode.telemetry.update();
            }
            runtime.reset();
            setDrivePower(0);


        }


    }

    // NOTE: left and right motors need to spin in opposite direction to move the robot in the same direction
    public void encoderDriveStrafe(double speed,
                                   double distance, String dir, double timeoutS) {
        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newLeftTargetFront;
        int newLeftTargetBack;
        int newRightTargetBack;
        int newRightTargetFront;

        if (op_mode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newRightTargetBack = rightDriveBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightTargetFront = rightDriveFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftTargetBack = leftDriveBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftTargetFront = leftDriveFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            //change the direction before setting the target position
            if (dir.equals("right")) {

                leftDriveBack.setDirection(DcMotor.Direction.REVERSE);//forward
                leftDriveFront.setDirection(DcMotor.Direction.FORWARD);//forward
                rightDriveBack.setDirection(DcMotor.Direction.REVERSE);//reverse
                rightDriveFront.setDirection(DcMotor.Direction.FORWARD);//reverse
            } else {
                leftDriveBack.setDirection(DcMotor.Direction.FORWARD);//forward
                leftDriveFront.setDirection(DcMotor.Direction.REVERSE);//forward
                rightDriveBack.setDirection(DcMotor.Direction.FORWARD);//reverse
                rightDriveFront.setDirection(DcMotor.Direction.REVERSE);//reverse
            }


            leftDriveFront.setTargetPosition((int) (newLeftTargetFront * 1.1));
            leftDriveBack.setTargetPosition((int) (newLeftTargetBack * 1.1));
            rightDriveFront.setTargetPosition((int) (newRightTargetFront * 1.1));
            rightDriveBack.setTargetPosition((int) (newRightTargetBack * 1.1));
            // Turn On RUN_TO_POSITION
            setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            setDrivePower(speed);
            runtime.reset();
            runtime.startTime();
            while (op_mode.opModeIsActive() &&
                    (leftDriveBack.isBusy() && rightDriveBack.isBusy() &&
                            leftDriveFront.isBusy() && rightDriveFront.isBusy()) && runtime.time()<timeoutS) {

                // Display it for the driver.
                op_mode.telemetry.addData("Path_1", "Running to %7d :%7d: %7d :%7d",
                        newLeftTargetFront, newRightTargetFront, newLeftTargetBack, newRightTargetBack);
                op_mode.telemetry.addData("Path_2", "Running at %7d :%7d: %7d : %7d",
                        leftDriveFront.getCurrentPosition(),
                        rightDriveFront.getCurrentPosition(),
                        leftDriveBack.getCurrentPosition(),
                        rightDriveBack.getCurrentPosition());
                op_mode.telemetry.addData("Time: ", runtime.time());

                op_mode.telemetry.update();
            }
            runtime.reset();
            setDrivePower(0);

            leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
            leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
            rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
            rightDriveFront.setDirection(DcMotor.Direction.REVERSE);

        }

    }

    public void setDrivePower(double power) {
        leftDriveFront.setPower(Math.abs(power));
        leftDriveBack.setPower(Math.abs(power));
        rightDriveFront.setPower(Math.abs(power));
        rightDriveBack.setPower(Math.abs(power));
    }
    public void driveToLine(boolean colorBlue) {
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setDrivePower(0.5);
        while (true) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            if (colors.blue > 0.75 && colorBlue || colors.red>0.75 && !colorBlue) {
                setDrivePower(0);
                break;

            }
            float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
            colors.red /= max;
            colors.green /= max;
            colors.blue /= max;
        }
        setDrivePower(0);
    }


/*  Direction for diagonal movement (45 degree)
    q2 | q1
    -- + --
    q3 | q4
 */
    public void encoderDriveStrafe45(double speed,
                                   double distance, String dir, double timeoutS) {
        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newLeftTargetFront;
        int newLeftTargetBack;
        int newRightTargetBack;
        int newRightTargetFront;

        if (op_mode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newRightTargetBack = rightDriveBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightTargetFront = rightDriveFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftTargetBack = leftDriveBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftTargetFront = leftDriveFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            //change the direction before setting the target position
            if (dir.equals("q2")) {
                leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
                rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
            }else if (dir.equals("q4")){
                leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
                rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
            }else if (dir.equals("q1")) {
                leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
                rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
            }else if (dir.equals("q3")){
                leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
                rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
            }

            leftDriveFront.setTargetPosition((int) (newLeftTargetFront * 1.1));
            leftDriveBack.setTargetPosition((int) (newLeftTargetBack * 1.1));
            rightDriveFront.setTargetPosition((int) (newRightTargetFront * 1.1));
            rightDriveBack.setTargetPosition((int) (newRightTargetBack * 1.1));
            // Turn On RUN_TO_POSITION
            setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // q2 and q4 use the same motors (similarly q1 and q3)
            if(dir.equals("q2") || dir.equals("q4")){
                leftDriveFront.setPower(Math.abs(0));
                leftDriveBack.setPower(Math.abs(speed));
                rightDriveFront.setPower(Math.abs(speed));
                rightDriveBack.setPower(Math.abs(0));
            }else {
                leftDriveFront.setPower(Math.abs(speed));
                leftDriveBack.setPower(Math.abs(0));
                rightDriveFront.setPower(Math.abs(0));
                rightDriveBack.setPower(Math.abs(speed));
            }

            // reset the timeout time and start motion.
            runtime.reset();
            runtime.startTime();
            while (op_mode.opModeIsActive() &&
                    ((leftDriveBack.isBusy() && rightDriveFront.isBusy() && (dir.equals("q2") || dir.equals("q4")))
                  || (leftDriveFront.isBusy() && rightDriveBack.isBusy() && (dir.equals("q1") || dir.equals("q3"))))
                    && runtime.time()<timeoutS) {

                // Display it for the driver.
                op_mode.telemetry.addData("Path_1", "Running to %7d :%7d: %7d :%7d",
                        newLeftTargetFront, newRightTargetFront, newLeftTargetBack, newRightTargetBack);
                op_mode.telemetry.addData("Path_2", "Running at %7d :%7d: %7d : %7d",
                        leftDriveFront.getCurrentPosition(),
                        rightDriveFront.getCurrentPosition(),
                        leftDriveBack.getCurrentPosition(),
                        rightDriveBack.getCurrentPosition());
                op_mode.telemetry.addData("Time: ", runtime.time());

                op_mode.telemetry.update();
            }
            runtime.reset();
            setDrivePower(0);

            leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
            leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
            rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
            rightDriveFront.setDirection(DcMotor.Direction.REVERSE);

        }

    }


    public void closeIntakeClaw(){
        claw1.setPosition(0.45);
        claw2.setPosition(0.5);
    }
    public void openIntakeClaw(){
        claw1.setPosition(0.75);
        claw2.setPosition(0.21);
    }
    public void releaseIntakeClaw(){
        claw1.setPosition(0.53);
        claw2.setPosition(0.43);
    }
    public void openFlipperClaw()
    {
        leftClamp.setPosition(0.01);
        rightClamp.setPosition(0.21);
    }
    public void closeFlipperClaw()
    {
        leftClamp.setPosition(0.13);
        rightClamp.setPosition(0.07);
    }
    public void frontFoundationClawUp()
    {
        rightFoundationClaw.setPosition(0.95);
        leftFoundationClaw.setPosition(0.8);
    }
    public void frontFoundationClawDown()
    {
        rightFoundationClaw.setPosition(0.8);
        leftFoundationClaw.setPosition(0.95);
    }
    public void backFoundationClawUp()
    {
        backFoundationClaw.setPosition(0.1);
    }
    public void backFoundationClawDown() {
        backFoundationClaw.setPosition(0.3);
    }
}





