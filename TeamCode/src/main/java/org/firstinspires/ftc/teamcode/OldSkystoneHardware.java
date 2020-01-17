
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


public class OldSkystoneHardware {
    /* Public OpMode members. */
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "AXx1coP/////AAAAGR5+EbHI40Yxm9qZuu+1B24qD4W8rX1Mpr3bUfT5zyb1j8gA4EQSLVMVxsL42LqB8fa6x/dQxnRpoAYen7mztKaCwY706bIMXjW/1n7YZHJ+/v4ZlzL39Bk5NHHMOvWrZ2nzRz2aE3ZM7dfUQFyOr0hIl5/cfmnSQ+MWUDJWynCWpinkTc8/CaHLh9f7O+jLy1zPzrS9nDJSj7GUXDRGS5+Aeh/Wwv0wvmfdz8ZdPA1zA0iXF1KqvfkjmMv25F15ikWX7/+XuwXlvHIEw1vruo9zbCaQcmgqve05tldl1y9aiMcdVeEJUrQLhdYA2ct8aKpqRIs6MXaw4otzbED5Zn93a4Wn0kcLBRUOQ0OJ6li0";

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
    public DcMotor arm = null;
    HardwareMap hwMap = null;
    public Servo claw = null;
    public Servo foundation1 = null;
    public Servo foundation2 = null;

    ElapsedTime runtime = new ElapsedTime();
    public NormalizedColorSensor colorSensor = null;
    public Servo foundationClaw = null;


    public OldSkystoneHardware() {


    }

    public void init(HardwareMap ahwMap, LinearOpMode opMode, String runeMode) {
        op_mode = opMode;
        hwMap = ahwMap;
        arm = hwMap.get(DcMotor.class, "arm");

        leftDriveFront = hwMap.get(DcMotor.class, "leftf");
        rightDriveFront = hwMap.get(DcMotor.class, "rightf");
        leftDriveBack = hwMap.get(DcMotor.class, "leftb");
        rightDriveBack = hwMap.get(DcMotor.class, "rightb");

        claw = hwMap.get(Servo.class, "claw");
        foundation1 = hwMap.get(Servo.class, "foundation1");
        foundation2 = hwMap.get(Servo.class, "foundation2");
        foundationClaw = hwMap.get(Servo.class, "foundationClaw");
        colorSensor = hwMap.get(NormalizedColorSensor.class, "sensor_color");


        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setPower(0);
        leftDriveFront.setPower(0);
        rightDriveFront.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
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

    public void encoderDrive(double speed,
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
            setDrivePower(Math.abs(speed));
            while (op_mode.opModeIsActive() &&
                    (leftDriveBack.isBusy() && rightDriveBack.isBusy() &&
                            leftDriveFront.isBusy() && rightDriveFront.isBusy())) {

                // Display it for the driver.
                op_mode.telemetry.addData("Path_1", "Running to %7d :%7d: %7d :%7d",
                        newLeftTargetFront, newRightTargetFront, newLeftTargetBack, newRightTargetBack);
                op_mode.telemetry.addData("Path_2", "Running at %7d :%7d: %7d : %7d",
                        leftDriveFront.getCurrentPosition(),
                        rightDriveFront.getCurrentPosition(),
                        leftDriveBack.getCurrentPosition(),
                        rightDriveBack.getCurrentPosition());
                op_mode.telemetry.update();
            }

            setDrivePower(0);


        }


    }

    public void encoderDriveStrafe(double speed,
                                   double distance, String dir) {
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
            while (op_mode.opModeIsActive() &&
                    (leftDriveBack.isBusy() && rightDriveBack.isBusy() &&
                            leftDriveFront.isBusy() && rightDriveFront.isBusy())) {

                // Display it for the driver.
                op_mode.telemetry.addData("Path_1", "Running to %7d :%7d: %7d :%7d",
                        newLeftTargetFront, newRightTargetFront, newLeftTargetBack, newRightTargetBack);
                op_mode.telemetry.addData("Path_2", "Running at %7d :%7d: %7d : %7d",
                        leftDriveFront.getCurrentPosition(),
                        rightDriveFront.getCurrentPosition(),
                        leftDriveBack.getCurrentPosition(),
                        rightDriveBack.getCurrentPosition());
                op_mode.telemetry.update();
            }
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

   public void moveArm(int position, double power) {
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
        //  while (op_mode.opModeIsActive() &&
        //            arm.isBusy()) {
        //       op_mode.telemetry.addData("Arm position:", arm.getCurrentPosition());
        //        op_mode.telemetry.update();
        //      }
        setDrivePower(0);
    }

    public void driveToLine(boolean colorBlue) {
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setDrivePower(0.6);
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
    }
}





