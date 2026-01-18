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
public class Blue extends LinearOpMode {
    private Follower follower;
    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose startPose = new Pose(34, 135, Math.toRadians(0));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        robot = new RobotHardware(this);
        robot.init();

        Paths myPaths = new Paths(follower);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                pedroDriveOnPathChain(myPaths.ReadMotif, 1, true),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.ShootPreload, 1, true),
                        robot.launcher.launcherOn(),
                        robot.susan.innerIntakeOn()
                ),
                robot.susan.ballKicker1.ballKickerUp(),
                robot.susan.ballKicker2.ballKickerUp(),
                robot.susan.ballKicker3.ballKickerUp(),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.Intake1, 0.5, true),
                        robot.intake.intakeOn(),
                        robot.susan.innerIntakeOff(),
                        robot.launcher.launcherOff()
                ),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.Shoot1, 1, true),
                        robot.launcher.launcherOn(),
                        robot.susan.innerIntakeOn()
                ),
                robot.intake.intakeOff(),
                robot.susan.ballKicker1.ballKickerUp(),
                robot.susan.ballKicker2.ballKickerUp(),
                robot.susan.ballKicker3.ballKickerUp(),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.Intake2, 0.5, true),
                        robot.intake.intakeOn(),
                        robot.susan.innerIntakeOff(),
                        robot.launcher.launcherOff()
                ),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.Shoot2, 1,true),
                    robot.launcher.launcherOn(),
                    robot.susan.innerIntakeOn()
                ),
                robot.intake.intakeOff(),
                robot.susan.ballKicker1.ballKickerUp(),
                robot.susan.ballKicker2.ballKickerUp(),
                robot.susan.ballKicker3.ballKickerUp(),
                robot.launcher.launcherOff()

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

        public PathChain ReadMotif;
        public PathChain ShootPreload;
        public PathChain Intake1;
        public PathChain Shoot1;
        public PathChain Intake2;
        public PathChain Shoot2;
        public PathChain LeaveShooting;

        public Paths(Follower follower) {
            ReadMotif = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(34.000, 135.000), new Pose(55.000, 126.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                    .build();

            ShootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.000, 126.000), new Pose(55.000, 89.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(305))
                    .build();

            Intake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.000, 89.000),
                                    new Pose(58.138, 78.932),
                                    new Pose(13.000, 80.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(305), Math.toRadians(0))
                    .build();

            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.000, 80.000), new Pose(55.000, 89.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                    .build();

            Intake2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.000, 89.000),
                                    new Pose(67.306, 51.652),
                                    new Pose(9.000, 56.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(0))
                    .build();

            Shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(9.000, 56.000), new Pose(55.000, 89.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                    .build();

            LeaveShooting = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.000, 89.000), new Pose(55.000, 55.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
                    .build();
        }
    }


}