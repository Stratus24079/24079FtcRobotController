package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public DcMotor intake = null;

    public enum IntakeMode {
        IN,
        OUT,
        OFF
    }

    public IntakeMode intakeMode = IntakeMode.OFF;
    public double INTAKE_SPEED = 0.7;

    public Intake(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        intake = myOpMode.hardwareMap.get(DcMotor.class, "intake");

        myOpMode.telemetry.addData(">", "Intake Initialized");
    }

    public void update() {
        if (intakeMode == IntakeMode.IN) {
            intake.setPower(INTAKE_SPEED);
        } else if (intakeMode == IntakeMode.OUT) {
            intake.setPower(-INTAKE_SPEED);
        } else {
            intake.setPower(0);
        }
    }

    public void teleOp() {
        update();

        if (myOpMode.gamepad2.left_bumper) {
            intakeMode = IntakeMode.IN;
        } else if (myOpMode.gamepad2.right_bumper) {
            intakeMode = IntakeMode.OUT;
        } else {
            intakeMode = IntakeMode.OFF;
        }
    }
}