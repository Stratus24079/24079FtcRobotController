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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueFar extends LinearOpMode {
    private Follower follower;
    private RobotHardware robot;


    int[] order = new int[]{2, 1, 0};

    @Override
    public void runOpMode() throws InterruptedException {
        Pose startPose = new Pose(85, 10, Math.toRadians(270));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        robot = new RobotHardware(this);
        robot.init();

        Paths myPaths = new Paths(follower);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                robot.launcher.scanMotif(),
                findOrder(),
                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.shootPreload, 1, true),
                        robot.launcher.launcherOn("FAR"),
                        robot.susan.innerIntakeOn()
                ),

                kickOrder(0),
                kickOrder(1),
                kickOrder(2)

                /*new ParallelAction(
                        pedroDriveOnPathChain(myPaths.intake1, 0.7, true),
                        robot.intake.intakeOn(),
                        robot.susan.innerIntakeOff(),
                        robot.launcher.launcherOff()

                ),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.shoot1, 1, true),
                        robot.launcher.launcherOn("FAR"),
                        robot.susan.innerIntakeOn()
                ),

                robot.susan.ballKickers[0].ballKickerUp(),
                robot.susan.ballKickers[1].ballKickerUp(),
                robot.susan.ballKickers[2].ballKickerUp(),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.intake2, 0.7, true),
                        robot.intake.intakeOn(),
                        robot.susan.innerIntakeOff(),
                        robot.launcher.launcherOff()
                ),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.shoot2, 1, true),
                        robot.launcher.launcherOn("FAR"),
                        robot.susan.innerIntakeOn()
                ),
                robot.intake.intakeOff(),

                robot.susan.ballKickers[0].ballKickerUp(),
                robot.susan.ballKickers[1].ballKickerUp(),
                robot.susan.ballKickers[2].ballKickerUp(),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.intake3, 0.7, true),
                        robot.intake.intakeOn(),
                        robot.susan.innerIntakeOff(),
                        robot.launcher.launcherOff()
                ),

                new ParallelAction(
                        pedroDriveOnPathChain(myPaths.shoot3, 1, true),
                        robot.launcher.launcherOn("FAR"),
                        robot.susan.innerIntakeOn()
                ),
                robot.intake.intakeOff(),

                robot.susan.ballKickers[0].ballKickerUp(),
                robot.susan.ballKickers[1].ballKickerUp(),
                robot.susan.ballKickers[2].ballKickerUp()*/
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



    public static class Paths {
        public PathChain shootPreload;
        public PathChain intake1;
        public PathChain shoot1;
        public PathChain intake2;
        public PathChain shoot2;
        public PathChain intake3;
        public PathChain shoot3;

        public Paths(Follower follower) {
            shootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.000, 10.000),

                                    new Pose(85.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))

                    .build();

            intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.000, 15.000),

                                    new Pose(135.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))

                    .build();

            shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.000, 15.000),

                                    new Pose(85.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))

                    .build();

            intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.000, 15.000),

                                    new Pose(135.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))

                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.000, 15.000),

                                    new Pose(85.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))

                    .build();

            intake3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.000, 15.000),

                                    new Pose(135.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))

                    .build();

            shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.000, 15.000),

                                    new Pose(85.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))

                    .build();
        }
    }
}
