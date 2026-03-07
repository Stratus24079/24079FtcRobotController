package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

public class Susan {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public NormalizedColorSensor colorSensor1 = null;
    public NormalizedColorSensor colorSensor2 = null;
    public NormalizedColorSensor colorSensor3 = null;
    public Servo RGBLight1 = null;
    public Servo RGBLight2 = null;
    public Servo RGBLight3 = null;
    final float[] hsvValues1 = new float[3];
    final float[] hsvValues2 = new float[3];
    final float[] hsvValues3 = new float[3];
    NormalizedRGBA colors1;
    NormalizedRGBA colors2;
    NormalizedRGBA colors3;
    float gain = 15;
    int loopCounter;

    public BallKicker[] ballKickers = null;
    public DcMotorEx innerMotor = null;

    public ElapsedTime timer = new ElapsedTime();
    public int kickerIndex = 0;
    public static final double KICK_TIME = 0.08;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    public Launcher launcher;

    //TODO Adjust based on desired states
    public enum SusanMode {
        MANUAL,
        SEQUENTIAL,
        SORTING
    }

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    //TODO Update values based on desired position
    public final double INNER_MOTOR_SPEED = 0.8;
    public Susan.SusanMode susanMode = SusanMode.MANUAL;

    //Constructor
    public Susan(LinearOpMode opmode, Launcher rpm) {
        myOpMode = opmode;
        launcher = rpm;
    }

    public void init() {
        colorSensor1 = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor2 = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
        colorSensor3 = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "colorSensor3");

        RGBLight1 = myOpMode.hardwareMap.get(Servo.class, "RGB1");
        RGBLight2 = myOpMode.hardwareMap.get(Servo.class, "RGB2");
        RGBLight3 = myOpMode.hardwareMap.get(Servo.class, "RGB3");

        colorSensor1.setGain(gain);
        colorSensor2.setGain(gain);
        colorSensor3.setGain(gain);

        if (colorSensor1 instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor1).enableLight(true);
        }
        if (colorSensor2 instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor2).enableLight(true);
        }
        if (colorSensor3 instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor3).enableLight(true);
        }

        ballKickers = new BallKicker[] {
                new BallKicker(myOpMode, "ballKicker1", 0.22, 0.07),//0.17, 0.06
                new BallKicker(myOpMode, "ballKicker2", 0.25, 0.1), //0.15, 0.05
                new BallKicker(myOpMode, "ballKicker3", 0.22, 0.07), //0.17, 0.07
        };
        innerMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "innerIntake");

        innerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        innerMotor.setDirection(DcMotor.Direction.REVERSE);

        for (BallKicker b : ballKickers) b.init();

        loopCounter = 0;

        myOpMode.telemetry.addData(">", "Susan Initialized");
    }

    public void update() {
        myOpMode.telemetry.addData("susanMode", susanMode);
        double innerMotorRPM = innerMotor.getVelocity() * 60 / 28;
        myOpMode.telemetry.addData("innerMotorRPM", innerMotorRPM);

        for (BallKicker b : ballKickers) b.update();

        RGBLight();
        /*
        myOpMode.telemetry.addData("rgb1", hsvValues1[0]);
        myOpMode.telemetry.addData("distance1", ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM));
        myOpMode.telemetry.addData("rgb2", hsvValues2[0]);
        myOpMode.telemetry.addData("distance2", ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM));
        myOpMode.telemetry.addData("rgb3", hsvValues3[0]);
        myOpMode.telemetry.addData("distance3", ((DistanceSensor) colorSensor3).getDistance(DistanceUnit.CM));
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor1.getNormalizedColors();
        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues1);
        myOpMode.telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        myOpMode.telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues1[0])
                .addData("Saturation", "%.3f", hsvValues1[1])
                .addData("Value", "%.3f", hsvValues1[2]);
        myOpMode.telemetry.addData("Alpha", "%.3f", colors.alpha);

        /* If this color sensor also has a distance sensor, display the measured distance.
         * Note that the reported distance is only useful at very close range, and is impacted by
         * ambient light and surface reflectivity. */
        /*
        if (colorSensor1 instanceof DistanceSensor) {
            myOpMode.telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM));
        }

         */
        loopCounter++;
        if(loopCounter > 20000){
            loopCounter = 0;
        }
    }

    public void RGBLight(){

     if(loopCounter % 5 ==0) {
         colors1 = colorSensor1.getNormalizedColors();
         Color.colorToHSV(colors1.toColor(), hsvValues1);
         if (hsvValues1[0] >= 180 && ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM) > 10 &&
                 ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM) < 25) {
             RGBLight1.setPosition(0.7);
         } else if (hsvValues1[0] > 120 && ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM) > 10 &&
                 ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM) < 25) {
             RGBLight1.setPosition(0.5);
         } else {
             RGBLight1.setPosition(0);
         }
     }else if(loopCounter % 5 == 1) {

         colors2 = colorSensor2.getNormalizedColors();
         Color.colorToHSV(colors2.toColor(), hsvValues2);
         if (hsvValues2[0] >= 180 && ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM) > 10 &&
                 ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM) < 25) {
             RGBLight2.setPosition(0.7);
         } else if (hsvValues2[0] > 120 && ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM) > 10 &&
                 ((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM) < 25) {
             RGBLight2.setPosition(0.5);
         } else {
             RGBLight2.setPosition(0);
         }
     }else if(loopCounter % 5 == 2) {

         colors3 = colorSensor3.getNormalizedColors();
         Color.colorToHSV(colors3.toColor(), hsvValues3);
         if (hsvValues3[0] >= 180 && ((DistanceSensor) colorSensor3).getDistance(DistanceUnit.CM) > 10 &&
                 ((DistanceSensor) colorSensor3).getDistance(DistanceUnit.CM) < 25) {
             RGBLight3.setPosition(0.7);
         } else if (hsvValues3[0] > 120 && ((DistanceSensor) colorSensor3).getDistance(DistanceUnit.CM) > 10 &&
                 ((DistanceSensor) colorSensor3).getDistance(DistanceUnit.CM) < 25) {
             RGBLight3.setPosition(0.5);
         } else {
             RGBLight3.setPosition(0);
         }
     }
    }

    public void teleOp() {
        update();
        //Set states based on gamepad presses
        if (susanMode == SusanMode.MANUAL) {
            if (myOpMode.gamepad2.x) {
                ballKickers[0].kickerMode = BallKicker.KickerMode.KICKER_UP;
            } else {
                ballKickers[0].kickerMode = BallKicker.KickerMode.KICKER_DOWN;
            }
            if (myOpMode.gamepad2.y) {
                ballKickers[1].kickerMode = BallKicker.KickerMode.KICKER_UP;
            } else {
                ballKickers[1].kickerMode = BallKicker.KickerMode.KICKER_DOWN;
            }
            if (myOpMode.gamepad2.b) {
                ballKickers[2].kickerMode = BallKicker.KickerMode.KICKER_UP;
            } else {
                ballKickers[2].kickerMode = BallKicker.KickerMode.KICKER_DOWN;
            }
        }

        else if(susanMode == SusanMode.SEQUENTIAL) {
            myOpMode.telemetry.addData("kickerindex", kickerIndex);

            double rpmError = launcher.revolutions_per_minute - launcher.measuredRPM;
            if(Math.abs(rpmError) < 50 && timer.seconds() > 0.2){
                if(kickerIndex < 3) {
                    ballKickers[kickerIndex].kickerMode = BallKicker.KickerMode.KICKER_UP;
                }
                kickerIndex++;
                timer.reset();
            }

            if(kickerIndex > 3){
                susanMode = SusanMode.MANUAL;
                ballKickers[0].kickerMode = BallKicker.KickerMode.KICKER_DOWN;
                ballKickers[1].kickerMode = BallKicker.KickerMode.KICKER_DOWN;
                ballKickers[2].kickerMode = BallKicker.KickerMode.KICKER_DOWN;
            }
        }

        if(myOpMode.gamepad2.x || myOpMode.gamepad2.y || myOpMode.gamepad2.b){
            susanMode = SusanMode.MANUAL;
        }

        if (myOpMode.gamepad2.a) {
                susanMode = SusanMode.SEQUENTIAL;
                kickerIndex = 0;
            }

            if (myOpMode.gamepad1.x) {
                innerMotor.setPower(INNER_MOTOR_SPEED);
            } else if (myOpMode.gamepad1.y) {
                innerMotor.setPower(0);
            }

            if (myOpMode.gamepad2.dpad_up) {
                innerMotor.setPower(INNER_MOTOR_SPEED);
            } else if (myOpMode.gamepad2.dpad_down) {
                innerMotor.setPower(-INNER_MOTOR_SPEED);
            } else if (myOpMode.gamepad2.dpad_left) {
                innerMotor.setPower(0);
            }
        }

    //use roadrunner actions in TeleOp
    public void kickSequential() {
        double t = timer.seconds();

        if (kickerIndex == 3) {
            if (ballKickers[2].isUp()) {
                susanMode = SusanMode.MANUAL;
            }
        } else {
            if (ballKickers[kickerIndex].isUp()) {
                ballKickers[(kickerIndex + 2) % 3].kickerMode = BallKicker.KickerMode.KICKER_DOWN;
            }

            if (t < KICK_TIME) {
                ballKickers[kickerIndex].kickerMode = BallKicker.KickerMode.KICKER_UP;
            } else {
                timer.reset();
                kickerIndex++;
            }
        }
    }

    public Action innerIntakeOn() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                innerMotor.setPower(INNER_MOTOR_SPEED);
                return false;
            }
        };
    }

    public Action innerIntakeOff() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                innerMotor.setPower(0);
                return false;
            }
        };
    }
}
