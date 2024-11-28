package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    //servos
    public Servo claw = null;
    public Servo clawJoint = null;
    public Servo pivot = null;

    public final double CLAW_OPEN = 0.12;
    public final double CLAW_CLOSED = 0;
    public final double PIVOT_UP = 0.3;
    public final double PIVOT_DOWN = 1;
    public final double PIVOT_GET_SPECIMEN = 0;
    public final double PIVOT_SPECIMEN_HIGH = 0.4;
    public final double CLAW_JOINT = 0.63;
    public final double CLAW_JOINT_GET_SPECIMEN = 0.2;
    public final double CLAW_JOINT_SPECIMEN_HIGH = 0.55;

    public double pivotPosition;
    public double clawPosition;
    public double clawJointPosition;

    public Scoring(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        clawJoint = myOpMode.hardwareMap.get(Servo.class, "clawJoint");
        pivot = myOpMode.hardwareMap.get(Servo.class, "pivot");

        claw.setPosition(CLAW_OPEN);
        pivot.setPosition(PIVOT_DOWN);
        clawJoint.setPosition(CLAW_JOINT);

        pivotPosition = PIVOT_DOWN;
        clawPosition = CLAW_OPEN;
        clawJointPosition = CLAW_JOINT;

        myOpMode.telemetry.addData(">", "Extension Initialized");
    }

    public void teleOp() {
        // send positions
        claw.setPosition(clawPosition);
        clawJoint.setPosition(clawJointPosition);
        pivot.setPosition(pivotPosition);

        // set positions
        if (myOpMode.gamepad1.left_bumper) {
            clawPosition = CLAW_OPEN;
        }

        if (myOpMode.gamepad1.right_bumper){
            clawPosition = CLAW_CLOSED;
        }

        if (myOpMode.gamepad2.right_bumper) {
            clawJointPosition = CLAW_JOINT_GET_SPECIMEN;
        }

        if (myOpMode.gamepad2.left_bumper) {
            clawJointPosition = CLAW_JOINT;
        }

        if (myOpMode.gamepad2.dpad_down) {
            pivotPosition = PIVOT_GET_SPECIMEN;
            clawJointPosition = CLAW_JOINT_GET_SPECIMEN;
            clawPosition = CLAW_OPEN;
        }

        if (myOpMode.gamepad2.dpad_left) {
            pivotPosition = PIVOT_GET_SPECIMEN;
            clawJointPosition = CLAW_JOINT_GET_SPECIMEN;
            clawPosition = CLAW_CLOSED;
        }

        if (myOpMode.gamepad2.dpad_up) {
            pivotPosition = PIVOT_SPECIMEN_HIGH;
            clawJointPosition = CLAW_JOINT_SPECIMEN_HIGH;
            clawPosition = CLAW_CLOSED;
        }

        if (myOpMode.gamepad2.dpad_right) {
            pivotPosition = PIVOT_GET_SPECIMEN + 0.2;
        }
    }

    public class ScoringPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pivot.setPosition(PIVOT_UP);
            claw.setPosition(CLAW_CLOSED);
            return false;
        }
    }
    public Action scoringPos() {
        return new ScoringPos();
    }

    public class ResetPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pivot.setPosition(PIVOT_DOWN);
            claw.setPosition(CLAW_OPEN);
            return false;
        }
    }
    public Action resetPos() {
        return new ResetPos();
    }

    public class OpenClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(CLAW_OPEN);
            return false;
        }
    }
    public Action openClaw() {
        return new ResetPos();
    }
}