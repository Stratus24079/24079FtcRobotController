package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class RobotHardware{
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    ElapsedTime runtime = new ElapsedTime();

    //TODO Add instance of Vision subsystem...either Webcam, HuskyLens, or Limelight
    public Drivetrain drivetrain;
    public BallKicker ballKicker;
    public Intake intake;
    public Launcher launcher;
    public Susan susan;

    String col = "red";

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {myOpMode = opmode;
    }

    public void init() {
        //drivetrain = new Drivetrain(myOpMode);
        intake = new Intake(myOpMode);
        launcher = new Launcher(myOpMode);
        susan = new Susan(myOpMode);

        //drivetrain.init();
        intake.init();
        launcher.init();
        susan.init();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
    }

    public void teleOp() {
        //drivetrain.teleOp();
        intake.teleOp();
        launcher.teleOp();
        susan.teleOp();
    }

    public void update(){
        intake.update();
        launcher.update();
        susan.update();
    }

    /*public void code(){
        print("Hello world!");

        Artifacts = true;
        Auto = works;
        Fail = no;
        Worlds = true;
    }
    */
}
