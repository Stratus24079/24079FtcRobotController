package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drivetrain {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;
    public IMU imu;

    static final double COUNTS_PER_INCH = 50;
    ElapsedTime timeout = new ElapsedTime();

    public Drivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {

        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData(">", "Drivetrain Initialized");
    }

    public void teleOp() {

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        double drive = -myOpMode.gamepad1.left_stick_y;
        double turn = myOpMode.gamepad1.right_stick_x;
        double strafe = -myOpMode.gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 2);

        frontLeftPower = (drive + turn - strafe) / denominator;
        frontRightPower = (drive - turn + strafe) / denominator;
        backLeftPower = (drive + turn + strafe) / denominator;
        backRightPower = (drive - turn - strafe) / denominator;

        leftFrontDrive.setPower(frontLeftPower * 1.3);
        rightFrontDrive.setPower(frontRightPower * 1.3);
        leftBackDrive.setPower(backLeftPower * 1.3);
        rightBackDrive.setPower(backRightPower * 1.3);
    }

    public void encoderTurn(float inches, double power) {
        final double WHEEL_DIAMETER = 4;
        final double COUNTS_PER_INCH = 537.6 / (Math.PI * WHEEL_DIAMETER);
        final int STRAIGHT_COUNTS = (int) (COUNTS_PER_INCH * inches * -1);
        while (Math.abs(rightFrontDrive.getCurrentPosition()) < Math.abs(STRAIGHT_COUNTS)) {
            turn(power);
            myOpMode.telemetry.addData("STRAIGHT_COUNTS", STRAIGHT_COUNTS);
            myOpMode.telemetry.addData("POSITION", rightFrontDrive.getCurrentPosition());
            myOpMode.telemetry.update();
        }
        stopMotors();

        return;
    }

    public void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void turn(double power) {
        rightFrontDrive.setPower(power);
        leftFrontDrive.setPower(-power);
        rightBackDrive.setPower(power);
        leftBackDrive.setPower(-power);
    }

    // TEMPORARY

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
}
