package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotHardware {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime timeout = new ElapsedTime();

    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor leftLift = null;
    public DcMotor rightLift = null;
    public DcMotor leftExtension = null;
    public DcMotor rightExtension = null;

    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {

        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftLift = myOpMode.hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = myOpMode.hardwareMap.get(DcMotor.class, "rightLift");
        leftExtension = myOpMode.hardwareMap.get(DcMotor.class, "leftExtension");
        rightExtension = myOpMode.hardwareMap.get(DcMotor.class, "rightExtension");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        myOpMode.telemetry.addData(">", "Initialized");
    }

    public void teleOp() {
        //Drivetrain TeleOp Code
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -myOpMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = myOpMode.gamepad1.left_stick_x;
        double yaw = myOpMode.gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double LEFT_FRONT_POWER = axial + lateral + yaw;
        double RIGHT_FRONT_POWER = axial - lateral - yaw;
        double LEFT_BACK_POWER = axial - lateral + yaw;
        double RIGHT_BACK_POWER = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(LEFT_FRONT_POWER), Math.abs(RIGHT_FRONT_POWER));
        max = Math.max(max, Math.abs(LEFT_BACK_POWER));
        max = Math.max(max, Math.abs(RIGHT_BACK_POWER));

        if (max > 1.0) {
            LEFT_FRONT_POWER /= max;
            RIGHT_FRONT_POWER /= max;
            LEFT_BACK_POWER /= max;
            RIGHT_BACK_POWER /= max;
        }

        leftFrontDrive.setPower(LEFT_FRONT_POWER*1.3);
        rightFrontDrive.setPower(RIGHT_FRONT_POWER*1.3);
        leftBackDrive.setPower(LEFT_BACK_POWER*1.3);
        rightBackDrive.setPower(RIGHT_BACK_POWER*1.3);

        // Linear slides
        rightLift.setPower(myOpMode.gamepad2.right_trigger-myOpMode.gamepad2.left_trigger);
        leftLift.setPower(myOpMode.gamepad2.right_trigger-myOpMode.gamepad2.left_trigger);

        // Horizontal extension
        rightExtension.setPower(myOpMode.gamepad1.right_trigger-myOpMode.gamepad1.left_trigger);
        leftExtension.setPower(myOpMode.gamepad1.right_trigger-myOpMode.gamepad1.left_trigger);
    }

    public void checkTelemetry(){
        myOpMode.telemetry.addData("LeftFront at", leftFrontDrive.getCurrentPosition());
        myOpMode.telemetry.addData("LeftBack at", leftBackDrive.getCurrentPosition());
        myOpMode.telemetry.addData("RightFront at", rightFrontDrive.getCurrentPosition());
        myOpMode.telemetry.addData("RightBack at", rightBackDrive.getCurrentPosition());
        myOpMode.telemetry.update();
    }
}