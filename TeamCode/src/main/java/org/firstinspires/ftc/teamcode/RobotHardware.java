package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime timeout = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_INCH = 50;

    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor leftLift = null;
    public DcMotor rightLift = null;
    public DcMotor leftExtension = null;
    public DcMotor rightExtension = null;
    public Servo pivot = null;
    public Servo claw = null;
    public Servo clawJoint = null;
    public Servo intake = null;
    public Servo intakeJoint = null;
    public IMU imu;

    public double CLAW_OPEN = 0.1;
    public double CLAW_CLOSED = 0;
    public double INTAKE_DOWN = 0.8;
    public double INTAKE_UP = 0.28;
    public double PIVOT_UP = 0.15;
    public double PIVOT_DOWN = 0.98;
    public double CLAW_JOINT = 0.31;

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
        pivot = myOpMode.hardwareMap.get(Servo.class, "pivot");
        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        clawJoint = myOpMode.hardwareMap.get(Servo.class, "clawJoint");
        intake = myOpMode.hardwareMap.get(Servo.class, "intake");
        intakeJoint = myOpMode.hardwareMap.get(Servo.class, "intakeJoint");
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        leftExtension.setDirection(DcMotor.Direction.FORWARD);
        rightExtension.setDirection(DcMotor.Direction.FORWARD);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

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

        leftFrontDrive.setPower(LEFT_FRONT_POWER * 1.3);
        rightFrontDrive.setPower(RIGHT_FRONT_POWER * 1.3);
        leftBackDrive.setPower(LEFT_BACK_POWER * 1.3);
        rightBackDrive.setPower(RIGHT_BACK_POWER * 1.3);

        // Linear slides
        leftLift.setPower(myOpMode.gamepad1.right_trigger - myOpMode.gamepad1.left_trigger);
        rightLift.setPower(myOpMode.gamepad1.right_trigger - myOpMode.gamepad1.left_trigger);

        // Horizontal extension
        rightExtension.setPower(myOpMode.gamepad2.right_trigger - myOpMode.gamepad2.left_trigger);
        leftExtension.setPower(myOpMode.gamepad2.right_trigger - myOpMode.gamepad2.left_trigger);

        // Claw + Pivot + Intake
        if (myOpMode.gamepad1.dpad_up) {
            intakeJoint.setPosition(INTAKE_DOWN);
            claw.setPosition(CLAW_CLOSED);
            pivot.setPosition(PIVOT_UP);
        } else if (myOpMode.gamepad1.dpad_down) {
            claw.setPosition(CLAW_OPEN);
            pivot.setPosition(PIVOT_DOWN);
        }

        // Claw joint
        clawJoint.setPosition(CLAW_JOINT);

        //if (myOpMode.gamepad2.dpad_up) {
        //    clawJoint.setPosition(1);
        //} else if (myOpMode.gamepad2.dpad_down){
        //    clawJoint.setPosition(0.9);
        //}

        // Claw
        if (myOpMode.gamepad1.left_bumper) {
            claw.setPosition(CLAW_OPEN);
        } else if (myOpMode.gamepad1.right_bumper){
            claw.setPosition(CLAW_CLOSED);
        }

        // Intake joint
        if (myOpMode.gamepad2.b) {
            intakeJoint.setPosition(INTAKE_DOWN);
        } else if (myOpMode.gamepad2.y) {
            intakeJoint.setPosition(INTAKE_UP);
        }

        if (myOpMode.gamepad2.left_bumper && intakeJoint.getPosition() <= 0.995 ) {
            intakeJoint.setPosition(intakeJoint.getPosition() + 0.005);
        } else if (myOpMode.gamepad2.right_bumper && intakeJoint.getPosition() >= 0.005 ) {
            intakeJoint.setPosition(intakeJoint.getPosition() - 0.005);
        }

        // Intake
        if (myOpMode.gamepad2.x) {
            intake.setPosition(1);
        } else if (myOpMode.gamepad2.a) {
            intake.setPosition(0);
        } else {
            intake.setPosition(0.5);
        }
    }

    public void encoderLift(double speed, int target, double timeoutS) {
        int targetCounts;

        // Ensure that the opmode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftLift.setTargetPosition(target);
            rightLift.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            timeout.reset();
            leftLift.setPower(Math.abs(speed));
            rightLift.setPower(Math.abs(speed));

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", target);
            myOpMode.telemetry.addData("Slides at", leftLift.getCurrentPosition());
            myOpMode.telemetry.addData("Slides at", rightLift.getCurrentPosition());
            myOpMode.telemetry.update();

            // Stop all motion;
            leftLift.setPower(0);
            rightLift.setPower(0);

            // Turn off RUN_TO_POSITION
            // Turn On RUN_TO_POSITION
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turnCCW (double motorPower, double degrees){
        // optionally reset imu heading - don't do this if you need field coordinates
        imu.resetYaw();
        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        //calculate the target angle based on current angle
        double targetAngle = orientation.getYaw(AngleUnit.DEGREES)+degrees;

        //start the motors
        leftFrontDrive.setPower(-motorPower);
        leftBackDrive.setPower(-motorPower);
        rightFrontDrive.setPower(motorPower);
        rightBackDrive.setPower(motorPower);

        //update orientation and check angle
        while (myOpMode.opModeIsActive() && targetAngle > orientation.getYaw(AngleUnit.DEGREES)){
            // Update Rotational Angles
            orientation = imu.getRobotYawPitchRollAngles();

            myOpMode.telemetry.addData("IMU position: ", orientation.getYaw(AngleUnit.DEGREES));
            myOpMode.telemetry.addData("Target Angle: ", targetAngle);
            myOpMode.telemetry.update();
        }

        //stop the motors
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void turnCW (double motorPower, double degrees){
        // optionally reset imu heading - don't do this if you need field coordinates
        imu.resetYaw();
        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        //calculate the target angle based on current angle
        double targetAngle = orientation.getYaw(AngleUnit.DEGREES)-degrees;

        //start the motors
        leftFrontDrive.setPower(motorPower);
        leftBackDrive.setPower(motorPower);
        rightFrontDrive.setPower(-motorPower);
        rightBackDrive.setPower(-motorPower);

        //update orientation and check angle
        while (myOpMode.opModeIsActive() && targetAngle < orientation.getYaw(AngleUnit.DEGREES)){
            // Update Rotational Angles
            orientation = imu.getRobotYawPitchRollAngles();

            myOpMode.telemetry.addData("IMU position: ", orientation.getYaw(AngleUnit.DEGREES));
            myOpMode.telemetry.addData("Target Angle: ", targetAngle);
        }

        //stop the motors
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void encoderDrive(double speed, double distance, double timeoutS) {
        int targetCounts;

        // Ensure that the opmode is still active
        if (myOpMode.opModeIsActive()) {

            //Reset the motor encoders so they start from 0
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            targetCounts = (int)(distance * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(targetCounts);
            leftBackDrive.setTargetPosition(targetCounts);
            rightFrontDrive.setTargetPosition(targetCounts);
            rightBackDrive.setTargetPosition(targetCounts);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            timeout.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (myOpMode.opModeIsActive() &&
                    (timeout.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy())) {

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to", targetCounts);
                myOpMode.telemetry.addData("LeftFront at", leftFrontDrive.getCurrentPosition());
                myOpMode.telemetry.addData("LeftBack at", leftBackDrive.getCurrentPosition());
                myOpMode.telemetry.addData("RightFront at", rightFrontDrive.getCurrentPosition());
                myOpMode.telemetry.addData("RightBack at", rightBackDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void autoStrafe (double speed, double distance, double timeoutS){
        int targetCounts;

        // Ensure that the opmode is still active
        if (myOpMode.opModeIsActive()) {

            //Reset the motor encoders so they start from 0
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Determine new target position, and pass to motor controller
        targetCounts = (int)(distance * COUNTS_PER_INCH);
        leftFrontDrive.setTargetPosition(targetCounts);
        leftBackDrive.setTargetPosition(-targetCounts);
        rightFrontDrive.setTargetPosition(-targetCounts);
        rightBackDrive.setTargetPosition(targetCounts);

        // Turn On RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        timeout.reset();
        leftFrontDrive.setPower(Math.abs(speed));
        leftBackDrive.setPower(Math.abs(speed));
        rightFrontDrive.setPower(Math.abs(speed));
        rightBackDrive.setPower(Math.abs(speed));

        while (myOpMode.opModeIsActive() && (timeout.seconds() < timeoutS) && (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {
            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", targetCounts);
            myOpMode.telemetry.addData("LeftFront at", leftFrontDrive.getCurrentPosition());
            myOpMode.telemetry.addData("LeftBack at", leftBackDrive.getCurrentPosition());
            myOpMode.telemetry.addData("RightFront at", rightFrontDrive.getCurrentPosition());
            myOpMode.telemetry.addData("RightBack at", rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.update();
        }

        // Stop all motion;
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        // Turn On RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void checkTelemetry(){
        myOpMode.telemetry.addData("LeftFront at", leftFrontDrive.getCurrentPosition());
        myOpMode.telemetry.addData("LeftBack at", leftBackDrive.getCurrentPosition());
        myOpMode.telemetry.addData("RightFront at", rightFrontDrive.getCurrentPosition());
        myOpMode.telemetry.addData("RightBack at", rightBackDrive.getCurrentPosition());
        myOpMode.telemetry.addData("\nLeftLift at", leftLift.getCurrentPosition());
        myOpMode.telemetry.addData("RightLift at", rightLift.getCurrentPosition());
        myOpMode.telemetry.addData("\nLeftExtension at", leftExtension.getCurrentPosition());
        myOpMode.telemetry.addData("RightExtension at", rightExtension.getCurrentPosition());
        myOpMode.telemetry.addData("\nPivot at", pivot.getPosition());
        myOpMode.telemetry.update();
    }
}