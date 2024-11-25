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

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public double CLAW_OPEN = 0.15;
    public double CLAW_CLOSED = 0;
    public double PIVOT_UP = 0.3;
    public double PIVOT_DOWN = 1;
    public double PIVOT_SPECIMEN = 0;
    public double PIVOT_SPECIMEN_HIGH = 0.5;
    public double CLAW_JOINT = 0.67;
    public double CLAW_JOINT_SPECIMEN = 0.2;

    public double extensionPosition;
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

        myOpMode.telemetry.addData("Extension: ", extensionPosition);
        myOpMode.telemetry.addData("Pivot: ", pivotPosition);
        myOpMode.telemetry.addData("Extension: ", extensionPosition);

        // set positions
        if (myOpMode.gamepad1.left_bumper) {
            clawPosition = CLAW_OPEN;
        }

        if (myOpMode.gamepad1.right_bumper){
            clawPosition = CLAW_CLOSED;
        }

        if (myOpMode.gamepad2.right_bumper) {
            clawJointPosition = CLAW_JOINT_SPECIMEN;
        }

        if (myOpMode.gamepad2.left_bumper) {
            clawJointPosition = CLAW_JOINT;
        }

        if (myOpMode.gamepad2.dpad_down) {
            pivotPosition = PIVOT_SPECIMEN;
            clawJointPosition = CLAW_JOINT_SPECIMEN;
            clawPosition = CLAW_OPEN;
        }

        if (myOpMode.gamepad2.dpad_up) {
            pivotPosition = PIVOT_SPECIMEN_HIGH;
        }
    }

    public class ScoringPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pivot.setPosition(PIVOT_UP);
            claw.setPosition(CLAW_CLOSED);
            myOpMode.telemetry.addData("Done", 0);
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