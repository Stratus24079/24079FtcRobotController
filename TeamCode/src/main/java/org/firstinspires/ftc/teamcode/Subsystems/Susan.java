package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Susan {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public BallKicker ballKicker1 = null;
    public BallKicker ballKicker2 = null;
    public BallKicker ballKicker3 = null;
    public CRServo innerServo = null;

    //TODO Adjust based on desired states
    public enum SusanMode {
        MANUAL,
        SEQUENTIAL,
        SORTING
    }

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    //TODO Update values based on desired position
    public final double INNER_SERVO_SPEED = 0.5;
    public Susan.SusanMode susanMode = SusanMode.MANUAL;

    //Constructor
    public Susan(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        ballKicker1 = new BallKicker(myOpMode, "ballKicker1", 0.06, 0.16);
        ballKicker2 = new BallKicker(myOpMode, "ballKicker2", 0.06, 0.16);
        ballKicker3 = new BallKicker(myOpMode, "ballKicker3", 0.1, 0.2);
        innerServo = myOpMode.hardwareMap.get(CRServo.class, "innerIntake");

        ballKicker1.init();
        ballKicker2.init();
        ballKicker3.init();

        myOpMode.telemetry.addData(">", "Susan Initialized");
    }

    public void update() {
        myOpMode.telemetry.addData("susanMode", susanMode);
        ballKicker1.update();
        ballKicker2.update();
        ballKicker3.update();
    }

    public void teleOp() {
        update();
        //Set states based on gamepad presses
        //TODO Update based on desired control scheme
        if (susanMode == SusanMode.MANUAL) {
            if (myOpMode.gamepad2.x) {
                ballKicker1.kickerMode = BallKicker.KickerMode.KICKER_UP;
            } else {
                ballKicker1.kickerMode = BallKicker.KickerMode.KICKER_DOWN;
            }
            if (myOpMode.gamepad2.y) {
                ballKicker2.kickerMode = BallKicker.KickerMode.KICKER_UP;
            } else {
                ballKicker2.kickerMode = BallKicker.KickerMode.KICKER_DOWN;
            }
            if (myOpMode.gamepad2.b) {
                ballKicker3.kickerMode = BallKicker.KickerMode.KICKER_UP;
            } else {
                ballKicker3.kickerMode = BallKicker.KickerMode.KICKER_DOWN;
            }

            if (myOpMode.gamepad2.dpad_up) {
                innerServo.setPower(INNER_SERVO_SPEED);
            } else if (myOpMode.gamepad2.dpad_down){
                innerServo.setPower(-INNER_SERVO_SPEED);
            } else if (myOpMode.gamepad2.dpad_left) {
                innerServo.setPower(0);
            }
        }
    }

    public Action innerIntakeOn() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    innerServo.setPower(INNER_SERVO_SPEED);
                    initialized = true;
                }
                return timer.seconds() < 1;
                //double vel = spin.getVelocity();
                //packet.put("shooterVelocity", vel);
                //return vel < 10_000.0;
            }
        };
    }

    public Action innerIntakeOff() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    innerServo.setPower(0);
                    initialized = true;
                }
                return timer.seconds() < 1;
            }
        };
    }
}
