package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Red extends LinearOpMode {
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose startPose = new Pose(111, 135, Math.toRadians(90));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        Paths myPaths = new Paths(follower);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(pedroDriveOnPathChain(myPaths.ShootPreload, 0.7, true));
        Actions.runBlocking(pedroDriveOnPathChain(myPaths.Intake1, 0.7, true));
        Actions.runBlocking(pedroDriveOnPathChain(myPaths.Shoot1, 0.7, true));
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

        public PathChain ShootPreload;
        public PathChain Intake1;
        public PathChain Shoot1;

        public Paths(Follower follower) {
            ShootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(111.000, 135.000), new Pose(88.000, 87.000))
                    )
                    .setLinearHeadingInterpolation(0, Math.toRadians(45))
                    .build();

            Intake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.000, 87.000), new Pose(104.685, 87.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(104.685, 87.000), new Pose(126.000, 87.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}