package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime runtime = new ElapsedTime();

    public Drivetrain drivetrain;
    public Scoring scoring;
    public Lift lift;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Robot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        drivetrain = new Drivetrain(myOpMode);
        scoring = new Scoring(myOpMode);
        lift = new Lift(myOpMode);

        drivetrain.init();
        scoring.init();
        lift.init();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void teleOp() {
        drivetrain.teleOp();
        lift.teleOp();
        scoring.teleOp();
    }

    public void deliver() {
        myOpMode.sleep(500);
        scoring.claw.setPosition(scoring.CLAW_OPEN);
        myOpMode.sleep(250);
    }

    public void retrieve() {
        myOpMode.sleep(250);
        myOpMode.sleep(500);
        scoring.claw.setPosition(scoring.CLAW_CLOSED);
        myOpMode.sleep(250);
        lift.liftToPositionPIDClass(750);
        myOpMode.sleep(400); //new one
    }

    public void toJunctionLeft() {
        runtime.reset();
        while (runtime.seconds() < 2.25) {
            //scoring.extension.setPosition(scoring.EXTENSION_IN);
            lift.liftToPositionPIDClass(3600);//3300
        }
    }

    public void toStackLeft(int liftHeight) {
        runtime.reset();
        while (runtime.seconds() < 1.75) {
            //scoring.extension.setPosition(scoring.EXTENSION_IN);
            //scoring.pivot.setPosition(scoring.CLAW_UP);
            lift.liftToPositionPIDClass(liftHeight);
        }
    }

    public void toJunctionRight() {
        runtime.reset();
        //scoring.pivot.setPosition(scoring.CLAW_HOVER);
        while (runtime.seconds() < 2.25) {
            lift.liftToPositionPIDClass(3600);
        }
    }

    public void toStackRight(int liftHeight) {
        runtime.reset();
        while (runtime.seconds() < 1.75) {
            //scoring.extension.setPosition(scoring.EXTENSION_IN);
            //scoring.pivot.setPosition(scoring.CLAW_UP);
            lift.liftToPositionPIDClass(liftHeight);
        }
    }
}