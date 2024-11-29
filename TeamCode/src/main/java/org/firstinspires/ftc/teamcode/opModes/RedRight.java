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

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "RedRight", group = "Concept")

public class RedRight extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // Add instance of RobotHardware
    Robot robot;

    // Create a runtime object so we can time loops
    ElapsedTime runtime = new ElapsedTime(0);

    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(10, -60, Math.toRadians(270));

        robot = new Robot(this);
        robot.init();

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder trajectory = drive.actionBuilder(initialPose)
                .stopAndAdd(robot.scoring.specimenHigh()) // Score preload
                .stopAndAdd(robot.intake.intakeDown())
                .strafeToLinearHeading(new Vector2d(10, -30), Math.toRadians(270))
                .stopAndAdd(robot.scoring.scoreSpecimen())
                .waitSeconds(0.5)
                .stopAndAdd(robot.scoring.resetPos())

                .setTangent(270)
                .splineToLinearHeading(new Pose2d(38, -16, 0), Math.PI / 2) // To 1st sample
                .stopAndAdd(robot.intake.intakeSpinIn())
                .strafeTo(new Vector2d(44, -16)) // Intake sample
                .stopAndAdd(robot.intake.intakeStop())
                .strafeToLinearHeading(new Vector2d(40, -55), Math.toRadians(315)) // To human player
                .stopAndAdd(robot.intakeSpinOutWithSensor()) // Outtake

                .strafeToLinearHeading(new Vector2d(54, -17), 0) // To 2nd sample
                .stopAndAdd(robot.intake.intakeSpinIn())
                .strafeTo(new Vector2d(58, -17)) // Intake sample
                .stopAndAdd(robot.intake.intakeStop())
                .strafeToLinearHeading(new Vector2d(40, -55), Math.toRadians(315)) // To human player
                .stopAndAdd(robot.intakeSpinOutWithSensor()) // Outtake

                .strafeToLinearHeading(new Vector2d(62, -17), 0) // To 3rd sample
                .stopAndAdd(robot.intake.intakeSpinIn())
                .strafeTo(new Vector2d(66, -17)) // Intake sample
                .stopAndAdd(robot.intake.intakeStop())
                .strafeToLinearHeading(new Vector2d(40, -55), Math.toRadians(315)) // To human player
                .stopAndAdd(robot.intakeSpinOutWithSensor())

                .stopAndAdd(robot.scoring.getSpecimen())
                .strafeToLinearHeading(new Vector2d(40, -45), Math.toRadians(90)) // Get 1st specimen
                .strafeTo(new Vector2d(40, -50))
                .stopAndAdd(robot.scoring.closeClaw())
                .stopAndAdd(robot.scoring.specimenHigh()) // Score 1st sample
                .strafeToLinearHeading(new Vector2d(10, -30), Math.toRadians(270))
                .stopAndAdd(robot.scoring.scoreSpecimen())

                .stopAndAdd(robot.scoring.getSpecimen())
                .strafeToLinearHeading(new Vector2d(40, -45), Math.toRadians(90)) // Get 2nd specimen
                .strafeTo(new Vector2d(40, -50))
                .stopAndAdd(robot.scoring.closeClaw())
                .stopAndAdd(robot.scoring.specimenHigh()) // Score 2nd sample
                .strafeToLinearHeading(new Vector2d(10, -30), Math.toRadians(270))
                .stopAndAdd(robot.scoring.scoreSpecimen())

                .stopAndAdd(robot.scoring.getSpecimen())
                .strafeToLinearHeading(new Vector2d(40, -45), Math.toRadians(90)) // Get 3rd specimen
                .strafeTo(new Vector2d(40, -50))
                .stopAndAdd(robot.scoring.closeClaw())
                .stopAndAdd(robot.scoring.specimenHigh()) // Score 3rd sample
                .strafeToLinearHeading(new Vector2d(10, -30), Math.toRadians(270))
                .stopAndAdd(robot.scoring.scoreSpecimen());

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryAction;
        trajectoryAction = trajectory.build();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    trajectoryAction
            );
        }
    }   // end runOpMode()
}   // end class