package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // lift motors
    public DcMotor extension = null;
    public Servo intake = null;
    public Servo intakeJoint = null;

    public final double INTAKE_DOWN = 0.8;
    public final double INTAKE_UP = 0.1;

    public double intakeJointPosition;

    public Intake(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        extension = myOpMode.hardwareMap.get(DcMotor.class, "leftExtension");
        intake = myOpMode.hardwareMap.get(Servo.class, "intake");
        intakeJoint = myOpMode.hardwareMap.get(Servo.class, "intakeJoint");

        // brake and encoders
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeJoint.setPosition(INTAKE_UP);

        intakeJointPosition = INTAKE_UP;

        myOpMode.telemetry.addData(">", "Intake Initialized");
    }

    public void teleOp() {
        extension.setPower(myOpMode.gamepad2.right_trigger - myOpMode.gamepad2.left_trigger);
        extension.setPower(myOpMode.gamepad2.right_trigger - myOpMode.gamepad2.left_trigger);

        // send positions
        intakeJoint.setPosition(intakeJointPosition);

        // set positions
        if (myOpMode.gamepad2.left_trigger > 0 && extension.getCurrentPosition() > -600
                || myOpMode.gamepad2.right_trigger > 0 && extension.getCurrentPosition() < -600) {
            intakeJoint.setPosition(INTAKE_UP);
        }

        if (myOpMode.gamepad2.b) {
            intakeJointPosition = INTAKE_DOWN;
        } else if (myOpMode.gamepad2.y) {
            intakeJointPosition = INTAKE_UP;
        }

        /* if (myOpMode.gamepad2.left_bumper && intakeJoint.getPosition() <= 0.995 ) {
            intakeJoint.setPosition(intakeJoint.getPosition() + 0.01);
        } else if (myOpMode.gamepad2.right_bumper && intakeJoint.getPosition() >= 0.005 ) {
            intakeJoint.setPosition(intakeJoint.getPosition() - 0.01);
        } */

        if (myOpMode.gamepad2.x /* && intakeJoint.getPosition() > INTAKE_DOWN - 0.2 */) {
            intake.setPosition(1);
        } else if (myOpMode.gamepad2.a) {
            intake.setPosition(0);
        } else {
            intake.setPosition(0.5);
        }
    }

    public class IntakeUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeJoint.setPosition(INTAKE_UP);
            return false;
        }
    }

    public Action intakeUp() {
        return new IntakeUp();
    }

    public class IntakeDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeJoint.setPosition(INTAKE_DOWN);
            return false;
        }
    }

    public Action intakeDown() {
        return new IntakeDown();
    }

    public class IntakeSpinIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPosition(1);
            return false;
        }
    }

    public Action intakeSpinIn() {
        return new IntakeSpinIn();
    }

    public class IntakeSpinOut implements Action {
        ElapsedTime runtime = new ElapsedTime(0);
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPosition(0);
            if (runtime.seconds() < 1) {
                return true;
            } else {
                intake.setPosition(0.5);
                return false;
            }
        }
    }

    public Action intakeSpinOut() {
        return new IntakeSpinOut();
    }

    public class IntakeStop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPosition(0.5);
            return false;
        }
    }

    public Action intakeStop() {
        return new IntakeStop();
    }
}