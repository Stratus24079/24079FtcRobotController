package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // lift motors
    public DcMotor rightLift = null;
    public DcMotor leftLift = null;

    PIDController leftLiftPID;
    PIDController rightLiftPID;

    // lift constants
    public static final double LIFT_KP = 0.01;
    public static final double LIFT_KI = 0;
    public static final double LIFT_KD = 0;
    public static final int LIFT_HIGH = 2300;
    public static final int LIFT_LOW = 1000;

    // if the subsystem has explicit states, it can be helpful to use an enum to define them
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
        leftLiftPID.setMaxOut(0.95);
        rightLiftPID.setMaxOut(0.95);

        rightLift = myOpMode.hardwareMap.get(DcMotor.class, "rightLift");
        leftLift = myOpMode.hardwareMap.get(DcMotor.class, "leftLift");

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
        if (Math.abs(myOpMode.gamepad1.right_trigger) > 0 || Math.abs(myOpMode.gamepad1.left_trigger) > 0) {
            liftMode = LiftMode.MANUAL;
        } /* else if (myOpMode.gamepad1.dpad_up) {
            liftMode = LiftMode.HIGH;
        } else if (myOpMode.gamepad1.dpad_down) {
            liftMode = LiftMode.LOW;
        } */

        if (liftMode == LiftMode.MANUAL) {
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftLift.setPower(myOpMode.gamepad1.left_trigger - myOpMode.gamepad1.right_trigger);
            rightLift.setPower(myOpMode.gamepad1.left_trigger - myOpMode.gamepad1.right_trigger);
        } else if (liftMode == LiftMode.HIGH) {
            liftToPositionPID(LIFT_HIGH);
        } else if (liftMode == LiftMode.LOW) {
            liftToPositionPID(LIFT_LOW);
        } else if (liftMode == LiftMode.GROUND) {
            liftToPositionPID(0);
        }
    }

    public void liftToPositionPID(double targetPosition) {
        double outLeft = leftLiftPID.calculate(targetPosition, leftLift.getCurrentPosition());
        double outRight = rightLiftPID.calculate(targetPosition, rightLift.getCurrentPosition());

        leftLift.setPower(outLeft);
        rightLift.setPower(outRight);

        myOpMode.telemetry.addData("LeftLift Power: ", outLeft);
        myOpMode.telemetry.addData("RightLift Power: ", outRight);
        myOpMode.telemetry.addData("LeftLift at: ", leftLift.getCurrentPosition());
        myOpMode.telemetry.addData("RightLift at: ", rightLift.getCurrentPosition());
        myOpMode.telemetry.update();
    }

    public void resetLiftPID() {
        leftLiftPID.reset();
        rightLiftPID.reset();
    }

    public Action liftToPosition(LiftMode liftMode) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (liftMode) {
                    case HIGH:
                        double outLeft = leftLiftPID.calculate(LIFT_HIGH, leftLift.getCurrentPosition());
                        double outRight = rightLiftPID.calculate(LIFT_HIGH, rightLift.getCurrentPosition());

                        leftLift.setPower(outLeft);
                        rightLift.setPower(outRight);

                        myOpMode.telemetry.addData("LeftLift Power: ", outLeft);
                        myOpMode.telemetry.addData("RightLift Power: ", outRight);
                        myOpMode.telemetry.addData("LeftLift at: ", leftLift.getCurrentPosition());
                        myOpMode.telemetry.addData("RightLift at: ", rightLift.getCurrentPosition());
                        myOpMode.telemetry.update();
                        break;
                    case LOW:
                        liftToPositionPID(LIFT_LOW);
                        break;
                    case GROUND:
                        liftToPositionPID(0);
                        break;
                }
                return false;
            }
        };
    }

    public class LiftHigh implements Action {
        ElapsedTime runtime = new ElapsedTime(0);

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double outLeft = leftLiftPID.calculate(LIFT_HIGH, leftLift.getCurrentPosition());
            double outRight = rightLiftPID.calculate(LIFT_HIGH, rightLift.getCurrentPosition());

            leftLift.setPower(outLeft);
            rightLift.setPower(outRight);

            myOpMode.telemetry.addData("LeftLift:", leftLift.getCurrentPosition());
            myOpMode.telemetry.addData("RightLift:", rightLift.getCurrentPosition());
            myOpMode.telemetry.update();

            if (runtime.seconds() > 3000) {
                leftLift.setPower(0);
                rightLift.setPower(0);
                myOpMode.telemetry.addData("Lift stopped", 0);
                myOpMode.telemetry.update();
                return false;
            }

            if (Math.abs(LIFT_HIGH - leftLift.getCurrentPosition()) > 30) {
                return true;
            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
                myOpMode.telemetry.addData("Lift high", 0);
                myOpMode.telemetry.update();
                return false;
            }
        }
    }
    public Action liftHigh() {
        return new LiftHigh();
    }

    public class LiftGround implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double outLeft = leftLiftPID.calculate(0, leftLift.getCurrentPosition());
            double outRight = rightLiftPID.calculate(0, rightLift.getCurrentPosition());

            leftLift.setPower(outLeft);
            rightLift.setPower(outRight);

            if (Math.abs(leftLift.getCurrentPosition()) > 10) {
                return true;
            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
                myOpMode.telemetry.addData("Lift ground", 0);
                return false;
            }
        }
    }
    public Action liftGround() {
        return new LiftGround();
    }
}