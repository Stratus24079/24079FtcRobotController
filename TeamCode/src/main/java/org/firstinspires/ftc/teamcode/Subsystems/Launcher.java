package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    private OpMode myOpMode = null;

    public DcMotorEx spin = null;
    public Servo hood = null;
    public double FAR = 0;
    public double CLOSE = 1;
    public double revolutions_per_minute = 5000;
    public static final double TICKS_PER_REVOLUTION = 28;
    double TICKS_PER_SECOND = revolutions_per_minute / 60 * TICKS_PER_REVOLUTION;

    public enum LauncherMode {
        ON,
        OFF,
        AUTO,
    }

    public enum HoodMode{
        CLOSE,
        FAR,
        AUTO,
    }

    public LauncherMode launcherMode = LauncherMode.OFF;
    public HoodMode hoodMode = HoodMode.CLOSE;

    public Launcher (OpMode opmode){
        myOpMode = opmode;
    }

    public void init(){
        spin = myOpMode.hardwareMap.get(DcMotorEx.class, "launcher");
        spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hood = myOpMode.hardwareMap.get(Servo.class, "hood");
        hood.setPosition(FAR);
    }

    public void update(){
        if (launcherMode == LauncherMode.ON) {
            spin.setVelocity(TICKS_PER_SECOND);
        } else if(launcherMode == LauncherMode.OFF) {
            spin.setVelocity(0);
        }

        if (hoodMode == HoodMode.FAR) {
            hood.setPosition(FAR);
        } else if(hoodMode == HoodMode.CLOSE) {
            hood.setPosition(CLOSE);
        }
    }

    public void teleOp(){
        update();

        if (myOpMode.gamepad1.y) {
            launcherMode = LauncherMode.ON;
        } else if (myOpMode.gamepad1.x) {
            launcherMode = LauncherMode.OFF;
        }
        if (myOpMode.gamepad1.a) {
            hoodMode = HoodMode.FAR;
        } else if(myOpMode.gamepad1.b) {
            hoodMode = HoodMode.CLOSE;
        }
    }

}
