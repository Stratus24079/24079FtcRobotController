package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private OpMode myOpMode = null;

    public DcMotorEx spin = null;
    public Servo hood = null;
    public double FAR = 0.25;
    public double CLOSE = 1;
    public double revolutions_per_minute = 5000; //og: 5000
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
        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spin.setDirection(DcMotor.Direction.REVERSE);

        hood = myOpMode.hardwareMap.get(Servo.class, "hood");
        hood.setPosition(CLOSE);
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

        if (myOpMode.gamepad1.x) {
            launcherMode = LauncherMode.ON;
        } else if (myOpMode.gamepad1.y) {
            launcherMode = LauncherMode.OFF;
        }
        if (myOpMode.gamepad1.a) {
            hoodMode = HoodMode.FAR;
        } else if(myOpMode.gamepad1.b) {
            hoodMode = HoodMode.CLOSE;
        }
    }

    public Action launcherOn() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    spin.setPower(0.85);
                    initialized = true;
                }
                return timer.seconds() < 2;
                //double vel = spin.getVelocity();
                //packet.put("shooterVelocity", vel);
                //return vel < 10_000.0;
            }
        };
    }

    public Action launcherOff() {
        return new Action() {
            ElapsedTime timer = new ElapsedTime();
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    spin.setVelocity(0);
                    initialized = true;
                }
                return timer.seconds() < 2;
                //double vel = spin.getVelocity();
                //packet.put("shooterVelocity", vel);
                //return vel < 10_000.0;
            }
        };
    }




}
