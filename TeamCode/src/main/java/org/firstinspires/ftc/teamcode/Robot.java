package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime runtime = new ElapsedTime();

    public Drivetrain drivetrain;
    public Scoring scoring;
    public Lift lift;
    public Intake intake;
    SensorREVColorDistance sensors;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        drivetrain = new Drivetrain(myOpMode);
        scoring = new Scoring(myOpMode);
        lift = new Lift(myOpMode);
        intake = new Intake(myOpMode);
        sensors = new SensorREVColorDistance(myOpMode);

        drivetrain.init();
        scoring.init();
        lift.init();
        intake.init();
        sensors.init();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void teleOp() {
        drivetrain.teleOp();
        lift.teleOp();
        scoring.teleOp();
        intake.teleOp();
        //sensors.getColor();
    }

    public class IntakeTransferWithSensor implements Action {
        ElapsedTime runtime = new ElapsedTime(0);

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.intake.setPosition(0);
            sensors.getColor();
            if (sensors.checkColorClaw(sensors.clawSensorColor) == 0) {
                return true;
            } else {
                intake.intake.setPosition(0.5);
                return false;
            }
        }
    }

    public Action intakeTransferWithSensor() {
        return new IntakeTransferWithSensor();
    }

    public class IntakeSpinOutWithSensor implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.intake.setPosition(0);
            if (sensors.checkColorIntake(sensors.intakeSensorColor) != 0) {
                return true;
            } else {
                intake.intake.setPosition(0.5);
                return false;
            }
        }
    }
    public Action intakeSpinOutWithSensor() {
        return new IntakeSpinOutWithSensor();
    }
}