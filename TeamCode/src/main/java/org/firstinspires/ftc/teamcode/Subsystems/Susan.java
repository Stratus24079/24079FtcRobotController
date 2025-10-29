package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

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
    public final double INNER_SERVO_SPEED = 1;
    public Susan.SusanMode susanMode = SusanMode.MANUAL;

    //Constructor
    public Susan(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        ballKicker1 = new BallKicker(myOpMode, "ballKicker1", 0.2, 0);
        ballKicker2 = new BallKicker(myOpMode, "ballKicker2", 0.2, 0);
        ballKicker3 = new BallKicker(myOpMode, "ballKicker3", 0.2, 0);
        innerServo = myOpMode.hardwareMap.get(CRServo.class, "innerServo");
        innerServo.setPower(INNER_SERVO_SPEED);

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
}
