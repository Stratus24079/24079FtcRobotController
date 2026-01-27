package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BallKicker {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public Servo kicker = null;

    //TODO Adjust based on desired states
    public enum KickerMode {
        KICKER_DOWN,
        KICKER_UP
    }

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    //TODO Update values based on desired position
    public double UP_POSITION = 0;
    public double DOWN_POSITION = 0;
    public String SERVO_NAME = "";
    public KickerMode kickerMode = KickerMode.KICKER_DOWN;

    //Constructor
    public BallKicker(LinearOpMode opmode, String servoName, double upIn, double downIn) {
        UP_POSITION = upIn;
        DOWN_POSITION = downIn;
        myOpMode = opmode;
        SERVO_NAME = servoName;
    }

    public void init() {
        kicker = myOpMode.hardwareMap.get(Servo.class, SERVO_NAME);
        kicker.setPosition(DOWN_POSITION);

        myOpMode.telemetry.addData(">", "Kicker Initialized");
    }

    public void update() {
        myOpMode.telemetry.addData("kickerMode", kickerMode);
        if (kickerMode == KickerMode.KICKER_UP) {
            kicker.setPosition(UP_POSITION);
        } else {
            kicker.setPosition(DOWN_POSITION);
        }
    }

    public void teleOp() {
        update();
        //Set states based on gamepad presses
        //TODO Update based on desired control scheme
    }

    public Action ballKickerUp() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    kicker.setPosition(UP_POSITION);
                    initialized = true;
                }
                if (timer.seconds() < 0.9) {
                    return true;
                } else {
                    kicker.setPosition(DOWN_POSITION);
                    return false;
                }
            }
        };
    }

    public Action ballKickerDown() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                kicker.setPosition(DOWN_POSITION);
                return false;
            }
        };
    }
}
