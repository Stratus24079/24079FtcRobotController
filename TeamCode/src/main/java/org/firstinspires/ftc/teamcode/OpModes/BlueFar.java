package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueFar extends LinearOpMode {
    private Follower follower;
    private RobotHardware robot;

    public double AUTO_CLOSE_RPM = 3280;
    public double AUTO_FAR_RPM = 4300;
    public Servo hood = null;

    int[] order = new int[]{2, 1, 0};

    @Override
    public void runOpMode() throws InterruptedException {
        hood.setPosition(0.7);

        Pose startPose = new Pose(60, 10, Math.toRadians(0));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        robot = new RobotHardware(this);
        robot.init();

        Paths myPaths = new Paths(follower);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                robot.launcher.scanMotif(),
                robot.launcher.switchBlue(),
                //findOrder(),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.shootPreload, 1, true),
                        robot.launcher.farHood(),
                        robot.launcher.launcherOn(AUTO_FAR_RPM),
                        robot.susan.innerIntakeOn()
                ),

                robot.launcher.autoAim(),

                kickOrder(0),
                new SleepAction(0.7),
                kickOrder(1),
                new SleepAction(0.7),
                kickOrder(2),

                new ParallelAction(
                        pedroDriveOnPathChainTime(myPaths.intake1, 1, true, 2),
                        robot.intake.intakeOn()
                ),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.slam1, 1, true)

                ),
                new ParallelAction(
                        pedroDriveOnPathChainTime(myPaths.slam2, 1, true,2)

                ),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.shoot1, 1, true),
                        robot.launcher.launcherOn(AUTO_FAR_RPM),
                        robot.susan.innerIntakeOn()
                ),

                robot.launcher.autoAim(),

                kickOrder(0),
                new SleepAction(0.7),
                kickOrder(1),
                new SleepAction(0.7),
                kickOrder(2),

                new ParallelAction(
                        pedroDriveOnPathChainTime(myPaths.intake2, 0.7, true, 2),
                        robot.susan.innerIntakeOff(),
                        robot.launcher.launcherOff()
                ),

                new ParallelAction(
                        pedroDriveOnPathChainTime(myPaths.slam3, 1, true,2)

                ),

                new ParallelAction(
                        pedroDriveOnPathChainTime(myPaths.slam4, 1, true,1)

                ),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.shoot2, 1, true),
                        robot.launcher.launcherOn(AUTO_FAR_RPM),
                        robot.intake.intakeOff(),
                        robot.susan.innerIntakeOn()
                ),

                robot.launcher.autoAim(),

                kickOrder(0),
                new SleepAction(0.7),
                kickOrder(1),
                new SleepAction(0.7),
                kickOrder(2),


                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.leave, 1, true),
                        robot.susan.innerIntakeOff(),
                        robot.launcher.launcherOff()
                )
        ));
    }

    public Action findOrder() {
        return new Action() {
            private boolean initialized = false;
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    timer.reset();
                }

                if (robot.launcher.id == 21) {
                    order = new int[]{0, 1, 2};
                } else if (robot.launcher.id == 22) {
                    order = new int[]{1, 0, 2};
                } else if (robot.launcher.id == 23) {
                    order = new int[]{2, 1, 0};
                }

                telemetry.addData("id", robot.launcher.id);
                telemetry.addData("order", order[0]);
                telemetry.addData("order", order[1]);
                telemetry.addData("order", order[2]);

                telemetry.update();

                return timer.seconds() < 0.5;
            }
        };
    }

    public Action kickOrder(int index) {
        return new Action() {
            private boolean initialized = false;
            private Action underlyingAction = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // LATE BINDING:
                    // We access "order" right now, ensuring we get the updated values
                    // from findOrder(), not the initial values.
                    int kickerId = order[index];

                    // Create the specific sub-action for this kicker
                    underlyingAction = robot.susan.ballKickers[kickerId].ballKickerUp();
                    initialized = true;
                }

                // Run the actual kicker action
                return underlyingAction.run(packet);
            }
        };
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

    private Action pedroDriveOnPathChainTime(PathChain targetPathChain, double maxPower, boolean holdPos, double time) {
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

                return follower.isBusy() && pathTimer.seconds() < time;
            }
        };
    }




    public static class Paths {
        public PathChain shootPreload;
        public PathChain intake1;
        public PathChain slam1;
        public PathChain slam2;
        public PathChain shoot1;
        public PathChain intake2;
        public PathChain slam3;
        public PathChain slam4;
        public PathChain shoot2;
        public PathChain leave;

        public Paths(Follower follower) {
            shootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 10.000),

                                    new Pose(60.000, 20.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 20.000),

                                    new Pose(9.571, 15.882)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            slam1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.571, 15.882),

                                    new Pose(32.224, 14.870)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            slam2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(32.224, 14.870),

                                    new Pose(8.534, 10.870)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))

                    .build();

            shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.534, 10.870),

                                    new Pose(60.000, 20.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 20.000),

                                    new Pose(9.441, 14.025)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))

                    .build();

            slam3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.441, 14.025),

                                    new Pose(32.460, 12.683)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(20))

                    .build();

            slam4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(32.460, 12.683),

                                    new Pose(9.888, 13.801)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(20))

                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.888, 13.801),

                                    new Pose(60.000, 20.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 20.000),

                                    new Pose(60.000, 35.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();
        }
    }

}
