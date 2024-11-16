package org.firstinspires.ftc.teamcode;

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
    public double CLAW_OPEN = 0.1;
    public double CLAW_CLOSED = 0;
    public double PIVOT_UP = 0;
    public double PIVOT_DOWN = 0.85;
    public double CLAW_JOINT = 1;

    public double extensionPosition;
    public double pivotPosition;
    public double clawPosition;

    public Scoring(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        claw = myOpMode.hardwareMap.get(Servo.class, "claw");
        clawJoint = myOpMode.hardwareMap.get(Servo.class, "clawJoint");
        pivot = myOpMode.hardwareMap.get(Servo.class, "pivot");

        claw.setPosition(CLAW_OPEN);
        pivot.setPosition(PIVOT_DOWN);

        pivotPosition = PIVOT_DOWN;
        clawPosition = CLAW_OPEN;

        myOpMode.telemetry.addData(">", "Extension Initialized");
    }

    public void teleOp() {
        // send positions
        claw.setPosition(clawPosition);
        clawJoint.setPosition(CLAW_JOINT);
        pivot.setPosition(pivotPosition);

        myOpMode.telemetry.addData("Extension: ", extensionPosition);

        // set positions
        if (myOpMode.gamepad1.left_bumper) {
            clawPosition = CLAW_OPEN;
        }

        if (myOpMode.gamepad1.right_bumper){
            clawPosition = CLAW_CLOSED;
        }

        if (myOpMode.gamepad1.dpad_up) {
            //intakeJoint.setPosition(INTAKE_DOWN - 0.2);
            clawPosition = CLAW_CLOSED;
            pivotPosition = PIVOT_UP;
        } else if (myOpMode.gamepad1.dpad_down) {
            clawPosition = CLAW_OPEN;
            pivotPosition = PIVOT_DOWN;
        }
    }
}