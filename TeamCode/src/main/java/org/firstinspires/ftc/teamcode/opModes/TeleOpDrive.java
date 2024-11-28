package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="TeleOp", group="Linear OpMode")

public class TeleOpDrive extends LinearOpMode {

    Robot robot = new Robot(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                robot.lift.liftMode = Lift.LiftMode.HIGH;
                robot.intake.intakeJoint.setPosition(robot.intake.INTAKE_DOWN);
                robot.scoring.clawPosition = robot.scoring.CLAW_CLOSED;
                robot.scoring.pivotPosition = robot.scoring.PIVOT_UP;
            }

            if (gamepad1.dpad_right) {
                robot.lift.liftMode = Lift.LiftMode.LOW;
                robot.intake.intakeJoint.setPosition(robot.intake.INTAKE_DOWN);
                robot.scoring.clawPosition = robot.scoring.CLAW_CLOSED;
                robot.scoring.pivotPosition = robot.scoring.PIVOT_UP;
            }

            if (gamepad1.dpad_left) {
                robot.intake.intakeJoint.setPosition(robot.intake.INTAKE_DOWN);
                robot.scoring.clawPosition = robot.scoring.CLAW_CLOSED;
                robot.scoring.pivotPosition = robot.scoring.PIVOT_UP;
            }

            if (gamepad1.dpad_down) {
                robot.lift.liftMode = Lift.LiftMode.GROUND;
                robot.intake.intakeJoint.setPosition(robot.intake.INTAKE_DOWN);
                robot.scoring.clawPosition = robot.scoring.CLAW_OPEN;
                robot.scoring.pivotPosition = robot.scoring.PIVOT_DOWN;
            }

            robot.teleOp();

            telemetry.addData("LeftFront at: ", robot.drivetrain.leftFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBack at: ", robot.drivetrain.leftBackDrive.getCurrentPosition());
            telemetry.addData("RightFront at: ", robot.drivetrain.rightFrontDrive.getCurrentPosition());
            telemetry.addData("RightBack at: " , robot.drivetrain.rightBackDrive.getCurrentPosition());
            telemetry.addData("LeftLift at: ", robot.lift.leftLift.getCurrentPosition());
            telemetry.addData("RightLift at: ", robot.lift.rightLift.getCurrentPosition());
            telemetry.addData("Extension at: ", robot.intake.extension.getCurrentPosition());
            telemetry.addData("Pivot at: ", robot.scoring.pivot.getPosition());
            telemetry.update();
        }
    }
}