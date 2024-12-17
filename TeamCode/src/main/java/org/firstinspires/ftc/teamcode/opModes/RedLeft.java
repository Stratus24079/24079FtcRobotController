/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "RedLeft", group = "Autonomous")

public class RedLeft extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // Add instance of RobotHardware
    Robot robot;

    // Create a runtime object so we can time loops
    ElapsedTime runtime = new ElapsedTime(0);

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-33, -60, Math.toRadians(270));

        robot = new Robot(this);
        robot.init();
        robot.scoring.claw.setPosition(robot.scoring.CLAW_CLOSED);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder trajectory = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-49, -41), Math.toRadians(45)) // Score preload
                .stopAndAdd(robot.scoring.closeClaw())
                .stopAndAdd(robot.intake.intakeDown())
                .stopAndAdd(robot.scoring.scoringPos())
                .stopAndAdd(robot.lift.liftHigh())
                .stopAndAdd(robot.scoring.resetPos())
                .waitSeconds(0.1)
                .stopAndAdd(robot.lift.liftGround())
                .stopAndAdd(robot.intake.intakeUp())

                .strafeToLinearHeading(new Vector2d(-28, -18), Math.toRadians(180)) // To 1st sample
                .stopAndAdd(robot.intake.intakeDown())
                .waitSeconds(0.4)
                .stopAndAdd(robot.intake.intakeSpinIn())
                .strafeTo(new Vector2d(-38, -18)) // Forward while intaking
                .stopAndAdd(robot.intake.intakeStop())
                .stopAndAdd(robot.intake.intakeUp())
                .waitSeconds(0.7)
                .stopAndAdd(robot.intakeTransferWithSensor()) // Outtake until color sensor detects in claw
                .stopAndAdd(robot.scoring.closeClaw())
                .stopAndAdd(robot.intake.intakeDown())
                .strafeToLinearHeading(new Vector2d(-47, -39), Math.toRadians(45)) // Score 1st sample
                .stopAndAdd(robot.scoring.scoringPos())
                .stopAndAdd(robot.lift.liftHigh())
                .stopAndAdd(robot.scoring.resetPos())
                .waitSeconds(0.1)
                .stopAndAdd(robot.lift.liftGround())

                .strafeToLinearHeading(new Vector2d(-40, -17), Math.toRadians(180)) // To 2nd sample
                .stopAndAdd(robot.intake.intakeSpinIn())
                .strafeTo(new Vector2d(-48, -17)) // Forward while intaking
                .stopAndAdd(robot.intake.intakeStop())
                .stopAndAdd(robot.intake.intakeUp())
                .waitSeconds(0.7)
                .stopAndAdd(robot.intakeTransferWithSensor()) // Outtake until color sensor detects in claw
                .stopAndAdd(robot.scoring.closeClaw())
                .stopAndAdd(robot.intake.intakeDown())
                .strafeToLinearHeading(new Vector2d(-46, -38), Math.toRadians(45)) // Score 2nd sample
                .stopAndAdd(robot.scoring.scoringPos())
                .stopAndAdd(robot.lift.liftHigh())
                .stopAndAdd(robot.scoring.resetPos())
                .waitSeconds(0.1)
                .stopAndAdd(robot.lift.liftGround())

                /*.strafeToLinearHeading(new Vector2d(-43, -16), Math.toRadians(180)) // To 3rd sample
                .stopAndAdd(robot.intake.intakeSpinIn())
                .strafeTo(new Vector2d(-56, -16)) // Forward while intaking
                .stopAndAdd(robot.intake.intakeStop())
                .strafeTo(new Vector2d(-53, -16))
                .stopAndAdd(robot.intake.intakeSpinIn())
                .waitSeconds(0.3)
                .stopAndAdd(robot.intake.intakeStop())
                .stopAndAdd(robot.intake.intakeUp())
                .waitSeconds(0.7)
                .stopAndAdd(robot.intakeTransferWithSensor()) // Outtake until color sensor detects in claw
                .stopAndAdd(robot.scoring.closeClaw())
                .stopAndAdd(robot.intake.intakeDown())
                .strafeToLinearHeading(new Vector2d(-44, -36), Math.toRadians(45)) // Score 3rd sample
                .stopAndAdd(robot.scoring.scoringPos())
                .stopAndAdd(robot.lift.liftHigh())
                .stopAndAdd(robot.scoring.resetPos())
                .waitSeconds(0.1)
                .stopAndAdd(robot.lift.liftGround())*/

                .strafeToLinearHeading(new Vector2d(-20, 10), Math.toRadians(180))
                .strafeTo(new Vector2d(-14, 0))
                .stopAndAdd(robot.scoring.scoringPos())
                .waitSeconds(5);

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryAction;
        trajectoryAction = trajectory.build();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    trajectoryAction
            );
        }
    }
}   // end class