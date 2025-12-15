package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="autoBlueLeft", group="Linear OpMode")
public class autoBlueLeft extends LinearOpMode {

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

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        robot.drivetrain.driveBack(),
                        robot.launcher.launcherOn(),
                        robot.susan.innerIntakeOn()
                ),
                robot.susan.ballKicker1.ballKickerUp(),
                robot.susan.ballKicker2.ballKickerUp(),
                robot.susan.ballKicker3.ballKickerUp(),
                robot.launcher.launcherOff(),
                robot.susan.innerIntakeOff()
        ));

        robot.drivetrain.turnCCW(0.5, 20);
        robot.drivetrain.autoStrafe(0.5, 60, 1);
    }
}