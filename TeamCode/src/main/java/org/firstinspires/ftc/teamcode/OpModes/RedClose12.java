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
public class RedClose12 extends LinearOpMode {
    private Follower follower;
    private RobotHardware robot;


    int[] order1 = new int[]{2, 1, 0};
    int[] order2 = new int[]{0, 1, 2};
    int[] order3 = new int[]{0,1,2};

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
                pedroDriveOnPathChain(myPaths.ReadMotif, 1, true),
                robot.launcher.scanMotif(),
                findOrder(),
                robot.launcher.switchRed(),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.ShootPreload, 1, true),

                        robot.launcher.launcherOn("CLOSE"),
                        robot.susan.innerIntakeOn()
                ),

                robot.launcher.autoAim(),

                kickOrder1(0),
                kickOrder1(1),
                kickOrder1(2),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.Intake1, 0.7, true),
                        robot.intake.intakeOn(),
                        robot.susan.innerIntakeOff(),
                        robot.launcher.launcherOff()

                ),
                new SequentialAction(
                        pedroDriveOnPathChain(myPaths.Gate, 0.7, true),
                        robot.intake.intakeOff()
                ),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.Shoot1, 1, true),
                        robot.launcher.launcherOn("CLOSE"),
                        robot.susan.innerIntakeOn()
                ),

                robot.launcher.autoAim(),

                kickOrder1(0),
                kickOrder1(1),
                kickOrder1(2),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.Intake2, 0.7, true),
                        robot.intake.intakeOn(),
                        robot.susan.innerIntakeOff(),
                        robot.launcher.launcherOff()
                ),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.Shoot2, 1, true),
                        robot.launcher.launcherOn("CLOSE"),
                        robot.susan.innerIntakeOn()
                ),
                robot.intake.intakeOff(),

                robot.launcher.autoAim(),

                kickOrder2(0),
                kickOrder2(1),
                kickOrder2(2),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.Intake3, 0.7, true),
                        robot.intake.intakeOn(),
                        robot.susan.innerIntakeOff(),
                        robot.launcher.launcherOff()
                ),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.Shoot3, 1, true),
                        robot.launcher.launcherOn("CLOSE"),
                        robot.susan.innerIntakeOn()
                ),
                robot.intake.intakeOff(),

                robot.launcher.autoAim(),

                kickOrder3(0),
                kickOrder3(1),
                kickOrder3(2),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.leave, 1, true),
                        robot.launcher.launcherOff(),
                        robot.susan.innerIntakeOff()
                )

        ));
    }

    public Action fireInOrder1() {
        return new Action() {
            private boolean initialized = false;
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                for (int i = 0; i < 3; i++) {
                    if (order1[i] == 1) robot.susan.ballKickers[0].ballKickerUp().run(packet);
                    else if (order1[i] == 2) robot.susan.ballKickers[1].ballKickerUp().run(packet);
                    else if (order1[i] == 3) robot.susan.ballKickers[2].ballKickerUp().run(packet);
                }
                return timer.seconds() < 2.5;
            }
        };
    }

    public Action fireInOrder2() {
        return new Action() {

            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                for (int i = 0; i < 3; i++) {
                    if (order2[i] == 1) robot.susan.ballKickers[0].ballKickerUp().run(packet);
                    else if (order2[i] == 2) robot.susan.ballKickers[1].ballKickerUp().run(packet);
                    else if (order2[i] == 3) robot.susan.ballKickers[2].ballKickerUp().run(packet);
                }
                return timer.seconds() < 0.5;
            }
        };
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
                    order1 = new int[]{0, 1, 2};
                    order2 = new int[]{1, 2, 0};
                    order3 = new int[]{2, 1, 0};
                } else if (robot.launcher.id == 22) {
                    order1 = new int[]{1, 0, 2};
                    order2 = new int[]{0, 1, 2};
                    order3 = new int[]{1, 2, 0};
                } else if (robot.launcher.id == 23) {
                    order1 = new int[]{2, 1, 0};
                    order2 = new int[]{0, 2, 1};
                    order3 = new int[]{0, 1, 2};
                }

                telemetry.addData("id", robot.launcher.id);
                telemetry.addData("order1", order1[0]);
                telemetry.addData("order1", order1[1]);
                telemetry.addData("order1", order1[2]);

                telemetry.addData("order2", order2[0]);
                telemetry.addData("order2", order2[1]);
                telemetry.addData("order2", order2[2]);

                telemetry.update();

                return timer.seconds() < 0.5;
            }
        };
    }

    public Action kickOrder1(int index) {
        return new Action() {
            private boolean initialized = false;
            private Action underlyingAction = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // LATE BINDING:
                    // We access "order1" right now, ensuring we get the updated values
                    // from findOrder(), not the initial values.
                    int kickerId = order1[index];

                    // Create the specific sub-action for this kicker
                    underlyingAction = robot.susan.ballKickers[kickerId].ballKickerUp();
                    initialized = true;
                }

                // Run the actual kicker action
                return underlyingAction.run(packet);
            }
        };
    }

    public Action kickOrder2(int index) {
        return new Action() {
            private boolean initialized = false;
            private Action underlyingAction = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // Access "order2" dynamically
                    int kickerId = order2[index];

                    underlyingAction = robot.susan.ballKickers[kickerId].ballKickerUp();
                    initialized = true;
                }

                return underlyingAction.run(packet);
            }
        };
    }

    public Action kickOrder3(int index) {
        return new Action() {
            private boolean initialized = false;
            private Action underlyingAction = null;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    // Access "order3" dynamically
                    int kickerId = order3[index];

                    underlyingAction = robot.susan.ballKickers[kickerId].ballKickerUp();
                    initialized = true;
                }

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
                telemetry.addData("order1", order1[0]);
                telemetry.addData("order1", order1[1]);
                telemetry.addData("order1", order1[2]);

                telemetry.addData("order2", order2[0]);
                telemetry.addData("order2", order2[1]);
                telemetry.addData("order2", order2[2]);

                telemetry.update();

                return follower.isBusy();
            }
        };
    }



    public static class Paths {
        public PathChain ReadMotif;
        public PathChain ShootPreload;
        public PathChain Intake1;
        public PathChain Gate;
        public PathChain Shoot1;
        public PathChain Intake2;
        public PathChain Shoot2;
        public PathChain Intake3;
        public PathChain Shoot3;
        public PathChain leave;

        public Paths(Follower follower) {
            ReadMotif = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(111.000, 135.000),

                                    new Pose(89.000, 125.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))

                    .build();

            ShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89.000, 125.000),

                                    new Pose(84.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(228))

                    .build();

            Intake1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.000, 94.000),
                                    new Pose(80.274, 84.075),
                                    new Pose(135.000, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(228), Math.toRadians(180))

                    .build();

            Gate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.000, 85.000),
                                    new Pose(112.920, 81.391),
                                    new Pose(130.000, 78.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.000, 78.000),

                                    new Pose(89.000, 89.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(220))

                    .build();

            Intake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(89.000, 89.000),
                                    new Pose(85.641, 56.795),
                                    new Pose(137.447, 61.224)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(180))

                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(137.447, 61.224),

                                    new Pose(89.000, 89.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(220))

                    .build();

            Intake3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(89.000, 89.000),
                                    new Pose(88.324, 31.304),
                                    new Pose(137.000, 37.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(180))

                    .build();

            Shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(137.000, 37.000),

                                    new Pose(89.000, 89.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(220))

                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89.000, 89.000),

                                    new Pose(89.000, 64.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(360))

                    .build();
        }
    }

}
