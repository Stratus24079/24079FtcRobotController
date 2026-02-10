package org.firstinspires.ftc.teamcode.OpModes;

import static java.lang.Math.abs;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Linear OpMode")
public class TeleOp extends LinearOpMode {

    //declaring instance of robot hardware
    RobotHardware robot;
    private Follower follower;
    public static Pose startingPose;
    public double speedMultiplier = 0.75;

    enum DriveMode{
        ROBOT_CENTRIC,
        RED_FIELD_CENTRIC,
        BLUE_FIELD_CENTRIC
    }
    DriveMode driveMode = DriveMode.ROBOT_CENTRIC;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(this);
        robot.init();

        robot.launcher.CLOSERPM = 3800;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.startTeleopDrive(true);
        follower.update();

        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            follower.update();
            robot.teleOp();
            telemetry.addData("Heading", follower.getPose().getHeading());

            if (robot.launcher.turretMode == Launcher.TurretMode.AUTO) {
                autoAim();
            }

            if (gamepad1.dpad_up) {
                driveMode = DriveMode.ROBOT_CENTRIC;
            } else if (gamepad1.dpad_left) {
               // driveMode = DriveMode.BLUE_FIELD_CENTRIC;
                robot.launcher.limelight.pipelineSwitch(1);
            } else if (gamepad1.dpad_right) {
                robot.launcher.limelight.pipelineSwitch(3);
               // driveMode = DriveMode.RED_FIELD_CENTRIC;
            }

            if (driveMode == DriveMode.ROBOT_CENTRIC) {
                follower.setTeleOpDrive(
                        gamepad1.left_stick_y * speedMultiplier,
                        gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier
                );
            } else if (driveMode == DriveMode.BLUE_FIELD_CENTRIC) {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * speedMultiplier,
                        -gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier,
                        false
                );
            } else if (driveMode == DriveMode.RED_FIELD_CENTRIC) {
                follower.setTeleOpDrive(
                        gamepad1.left_stick_y * speedMultiplier,
                        gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier,
                        false
                );
            }
            if(gamepad1.right_bumper){
                speedMultiplier = 1;
            }
            else if(gamepad1.left_bumper){
                speedMultiplier = 0.25;
            }
            else{
                speedMultiplier = 0.75;
            }
            telemetry.addData("Drive Mode: ", driveMode);
            telemetry.update();
        }
    }

    public void autoAim() {
        double robotHeading = Math.toDegrees(follower.getHeading());
        double turretAngle = turretToDeg(robot.launcher.turretEncoder.getVoltage());

        double targ = 0;
        double turretTarget = wrap(targ - robotHeading);

        double error = wrap(turretTarget - turretAngle);

        double power = robot.launcher.turretPID.calculate(error);

        if (abs(error) > 1.0) {
            robot.launcher.turret.setPower(power);
        } else {
            robot.launcher.turret.setPower(0);
        }

        telemetry.addData("turretPower", power);
        telemetry.addData("turretDeg", turretAngle);
        telemetry.addData("robotHead", robotHeading);
        telemetry.addData("error", error);
    }

    private double turretToDeg(double v) {
        return wrap(v / 3.3 * 360);
    }

    private double wrap(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }
}