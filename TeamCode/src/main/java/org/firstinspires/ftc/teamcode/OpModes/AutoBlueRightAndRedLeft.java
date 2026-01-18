package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoBlueRightAndRedLeft", group="Linear OpMode")
public class AutoBlueRightAndRedLeft extends LinearOpMode {
    private Follower follower;
    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose startPose = new Pose(56, 8, Math.toRadians(90));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        robot = new RobotHardware(this);
        robot.init();

        Paths myPaths = new Paths(follower);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                pedroDriveOnPathChain(myPaths.Leave, 1, true)
        ));
    }

    private Action pedroDriveOnPathChain(PathChain targetPathChain, double maxPower, boolean holdPos) {
        return new Action() {
            private boolean initialized = false;
            ElapsedTime pathTimer = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    pathTimer.reset();
                    follower.followPath(targetPathChain, maxPower, holdPos);
                }

                follower.update();

                telemetry.addData("x", follower.getPose().getX());
                telemetry.addData("y", follower.getPose().getY());
                telemetry.addData("heading", follower.getPose().getHeading());

                telemetry.update();

                return follower.isBusy();
            }
        };
    }

    public static class Paths {

        public PathChain Leave;

        public Paths(Follower follower) {
            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 30.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();
        }
    }

}