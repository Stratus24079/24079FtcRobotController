package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Red extends LinearOpMode {
    private Follower follower;
    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose startPose = new Pose(111, 135, Math.toRadians(180));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        robot = new RobotHardware(this);
        robot.init();

        Paths myPaths = new Paths(follower);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.ShootPreload, 0.7, true),
                        robot.launcher.launcherOn(),
                        robot.susan.innerIntakeOn()
                ),
                robot.susan.ballKicker1.ballKickerUp(),
                robot.susan.ballKicker2.ballKickerUp(),
                robot.susan.ballKicker3.ballKickerUp(),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.Intake1, 0.7, true),
                        robot.intake.intakeOn(),
                        robot.susan.innerIntakeOff(),
                        robot.launcher.launcherOff()
                ),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.Shoot1, 0.7, true),
                        robot.launcher.launcherOn(),
                        robot.intake.intakeOff(),
                        robot.susan.innerIntakeOn()
                ),
                robot.susan.ballKicker1.ballKickerUp(),
                robot.susan.ballKicker2.ballKickerUp(),
                robot.susan.ballKicker3.ballKickerUp(),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.Intake2, 0.7, true),
                        robot.intake.intakeOn(),
                        robot.susan.innerIntakeOff(),
                        robot.launcher.launcherOff()
                )
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

        public PathChain ShootPreload;
        public PathChain Intake1;
        public PathChain Shoot1;
        public PathChain Intake2;

        public Paths(Follower follower) {
            ShootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(111.000, 135.000), new Pose(82.000, 85.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Intake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(82.000, 85.000), new Pose(135.000, 85.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(135.000, 85.000), new Pose(82.000, 85.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Intake2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.000, 85.000),
                                    new Pose(77.000, 57.000),
                                    new Pose(135.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }

}