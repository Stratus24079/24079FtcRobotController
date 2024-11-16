package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lift {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // lift motors
    public DcMotor rightLift = null;
    public DcMotor leftLift = null;

    PIDController leftLiftPID;
    PIDController rightLiftPID;

    //touch sensor
    public TouchSensor touch = null;

    //lift constants
    public static final double LIFT_KP = 0.01;
    public static final double LIFT_KI = 0;
    public static final double LIFT_KD = 0;

    //if the subsystem has explicit states, it can be helpful to use an enum to define them
    public enum LiftMode {
        MANUAL,
        HIGH,
        LOW,
        GROUND
    }

    public LiftMode liftMode = LiftMode.MANUAL;

    public Lift(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        leftLiftPID = new PIDController(LIFT_KP, LIFT_KI, LIFT_KD);
        rightLiftPID = new PIDController(LIFT_KP, LIFT_KI, LIFT_KD);
        leftLiftPID.maxOut = 0.95;
        rightLiftPID.maxOut = 0.95;

        rightLift = myOpMode.hardwareMap.get(DcMotor.class, "liftRight");
        leftLift = myOpMode.hardwareMap.get(DcMotor.class, "liftLeft");

        //touch = myOpMode.hardwareMap.get(TouchSensor.class, "touch");

        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        // brake and encoders
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        myOpMode.telemetry.addData(">", "Lift Initialized");
    }

    public void teleOp() {
        // gamepad control specific to lift
        if (Math.abs(myOpMode.gamepad1.right_trigger) > 0.3 || Math.abs(myOpMode.gamepad1.left_trigger) > 0.3) {
            liftMode = LiftMode.MANUAL;
        } else if (myOpMode.gamepad1.dpad_up) {
            liftMode = LiftMode.HIGH;
        } else if (myOpMode.gamepad1.dpad_down) {
            liftMode = LiftMode.LOW;
        }

        myOpMode.telemetry.addData("touch", "Pressed: " + touch.isPressed());
        //if (touch.isPressed()) {
        //    leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //    rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //}

        // code defining behavior of lift in each state
        if (liftMode == LiftMode.MANUAL) {
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftLift.setPower(myOpMode.gamepad1.right_trigger - myOpMode.gamepad1.left_trigger);
            rightLift.setPower(myOpMode.gamepad1.right_trigger - myOpMode.gamepad1.left_trigger);

            //if (Math.abs(myOpMode.gamepad2.right_stick_y) > 0.1) {
                //robot.liftLeft.setPower(-0.8);
                //robot.liftRight.setPower(-0.8);
                //leftLift.setPower(-myOpMode.gamepad2.right_stick_y);
                //rightLift.setPower(-myOpMode.gamepad2.right_stick_y);
            //} else {
                //leftLift.setPower(0);
                //rightLift.setPower(0);
            //}
        } else if (liftMode == LiftMode.HIGH) {
            liftToPositionPIDClass(3700);
        } else if (liftMode == LiftMode.LOW) {
            liftToPositionPIDClass(1409);
        } else if (liftMode == LiftMode.GROUND) {
            resetLift(-0.8);
        }
    }

    // functions specific to lift
    public void liftToPositionPIDClass(double targetPosition) {
        double outLeft = leftLiftPID.calculate(targetPosition, leftLift.getCurrentPosition());
        double outRight = rightLiftPID.calculate(targetPosition, rightLift.getCurrentPosition());

        leftLift.setPower(outLeft);
        rightLift.setPower(outRight);

        myOpMode.telemetry.addData("LiftLeftPower: ", outLeft);
        myOpMode.telemetry.addData("LiftRightPower: ", outRight);
    }

    public void resetLift(double speed) {
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //if (touch.isPressed() == false) {
        //    leftLift.setPower(speed);
        //    rightLift.setPower(speed);
        //} else {
        //    leftLift.setPower(0);
        //    rightLift.setPower(0);
        //}
    }

    public void resetLiftPID() {
        leftLiftPID.reset();
        rightLiftPID.reset();
    }
}