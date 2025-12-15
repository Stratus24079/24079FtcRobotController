package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoBlueRightAndRedLeft", group="Linear OpMode")
public class AutoBlueRightAndRedLeft extends LinearOpMode {

    //declaring instance of robot hardware
    RobotHardware robot;


    @Override
    public void runOpMode() {
        //calling constructor
        robot = new RobotHardware(this);
        //calling init function
        robot.init();
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        robot.drivetrain.encoderDrive(0.5, -60, 0.2);
    }
}