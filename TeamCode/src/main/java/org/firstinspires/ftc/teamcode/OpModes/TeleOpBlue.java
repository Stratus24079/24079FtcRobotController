package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpBlue", group="Linear OpMode")
public class TeleOpBlue extends LinearOpMode {

    //declaring instance of robot hardware
    RobotHardware robot;


    @Override
    public void runOpMode() {
        //calling constructor
        robot = new RobotHardware(this);
        //calling init function
        robot.init("blue");
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.teleOp();
            telemetry.update();
        }
    }
}