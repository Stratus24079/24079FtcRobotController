package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utility.PIDController;

import java.util.ArrayList;
import java.util.List;

public class Launcher {
    // pipeline
    // 0 = obelisk
    // 1 = Blue 2D
    // 2 = Blue 3D
    // 3 = Red 2D
    // 4 = Red 3D
    private OpMode myOpMode = null;
    public Limelight3A limelight;
    public DcMotorEx spin1 = null;
    public DcMotorEx spin2 = null;
    public Servo hood = null;
    public CRServo turret = null;
    public AnalogInput turretEncoder = null;
    public PIDController turretPID = null;

    public double FAR = 0.7;
    public double CLOSE = 1;
    public double FARRPM = 4500;
    public double CLOSERPM = 3200;
    public double AUTO_CLOSE_RPM = 3330;
    public double AUTO_FAR_RPM = 6000;
    public double measuredRPM;

    public double MIN_TURRET_SPEED = 0.1;
    public double revolutions_per_minute = 5000;
    public static final double TICKS_PER_REVOLUTION = 28;
    double TICKS_PER_SECOND = revolutions_per_minute / 60 * TICKS_PER_REVOLUTION;
    public static double turretKP = 0.01;
    public double turretKI = 0;
    public double turretKD = 0;
    public int id = 21;
    LLResult result;
    List<LLResultTypes.FiducialResult> fiducialResults;
    double distance;

    //turret testing
    private double lastRawVoltage = 0;
    private int turnCount = 0;
    private double totalUnwrappedDegrees = 0;
    private final double MAX_VOLTAGE = 3.3;
    private final double WRAP_THRESHOLD = 2.0; // Voltage jump to detect a turn
    private final double LIMIT_MIN_DEG = -470;
    private final double LIMIT_MAX_DEG = 420;

    //tuning
    public static double hoodTuning = 1;
    public static int TUNINGRPM = 2000;

    public enum LauncherMode {
        ON,
        OFF,
        AUTO,
    }

    public enum HoodMode{
        CLOSE,
        FAR,
        AUTO,
        TUNING,
    }

    public enum TurretMode{
        MANUAL,
        AUTO,
    }
    public LauncherMode launcherMode = LauncherMode.OFF;
    public HoodMode hoodMode = HoodMode.CLOSE;
    public TurretMode turretMode = TurretMode.MANUAL;

    public Launcher (OpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){
        spin1 = myOpMode.hardwareMap.get(DcMotorEx.class, "launcher");
        spin1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spin1.setDirection(DcMotor.Direction.REVERSE);

        spin2 = myOpMode.hardwareMap.get(DcMotorEx.class, "launcher2");
        spin2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //spin2.setDirection(DcMotor.Direction.REVERSE);

        hood = myOpMode.hardwareMap.get(Servo.class, "hood");
        hood.setPosition(CLOSE);

        turret = myOpMode.hardwareMap.get(CRServo.class, "turret");
        turret.setPower(0);

        turretEncoder = myOpMode.hardwareMap.get(AnalogInput.class, "turretEncoder");

        turretPID = new PIDController(turretKP, turretKI, turretKD, 0.8);

        limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        limelight.start();
        result = limelight.getLatestResult();
        fiducialResults = result.getFiducialResults();
    }

    public void update(){
        TICKS_PER_SECOND = (revolutions_per_minute / 60) * TICKS_PER_REVOLUTION;

        if (launcherMode == LauncherMode.ON) {
            spin1.setVelocity(TICKS_PER_SECOND);
            spin2.setVelocity(TICKS_PER_SECOND);
        } else if(launcherMode == LauncherMode.OFF) {
            spin1.setVelocity(0);
            spin2.setVelocity(0);
        }

        if (hoodMode == HoodMode.FAR) {
            hood.setPosition(FAR);
            revolutions_per_minute = FARRPM;
        } else if(hoodMode == HoodMode.CLOSE) {
            hood.setPosition(CLOSE);
            revolutions_per_minute =  CLOSERPM;
        } else if(hoodMode == HoodMode.TUNING){
            hood.setPosition(hoodTuning);
            revolutions_per_minute = TUNINGRPM;
        } else if(hoodMode == HoodMode.AUTO){
            revolutions_per_minute = 666 * distance + 2373;
            if(distance > 2.4){
                hood.setPosition(0.7);
            }
            else{
                hood.setPosition(1);
            }
        }

        //turret testing
            double currentVoltage = turretEncoder.getVoltage();
            double delta = currentVoltage - lastRawVoltage;

            // Detect wrap-around using a threshold (approx 80% of max voltage)
            // If it jumps from 0.2 to 3.1, that's a negative wrap
            if (delta > (MAX_VOLTAGE * 0.8)) {
                turnCount--;
            }
            // If it jumps from 3.1 to 0.2, that's a positive wrap
            else if (delta < -(MAX_VOLTAGE * 0.8)) {
                turnCount++;
            }

            double currentRotationDeg = (currentVoltage / MAX_VOLTAGE) * 360.0;
            totalUnwrappedDegrees = (turnCount * 360.0) + currentRotationDeg;
            myOpMode.telemetry.addData("turretDegrees", totalUnwrappedDegrees);

            // 200 start
            // 420 limit
            // -470 limit

            lastRawVoltage = currentVoltage;

            //Limits: -807
            //end of turret test

        //if(turretMode == TurretMode.MANUAL && !(totalUnwrappedDegrees > 420 && myOpMode.gamepad2.right_stick_x < 0 || totalUnwrappedDegrees < -470 && myOpMode.gamepad2.right_stick_x > 0)) {
        if(turretMode == TurretMode.MANUAL) {
                turret.setPower(myOpMode.gamepad2.right_stick_x);
            } else if (turretMode == TurretMode.AUTO) {
                if(Math.abs(myOpMode.gamepad2.right_stick_x) > 0.2){
                    turretMode = TurretMode.MANUAL;
                }
                result = limelight.getLatestResult();
                if (result.isValid()) {

                /*
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();

                myOpMode.telemetry.addData("txnc", result.getTxNC());
                myOpMode.telemetry.addData("ty", result.getTy());
                myOpMode.telemetry.addData("tync", result.getTyNC());

                myOpMode.telemetry.addData("Botpose", botpose.toString());
                */

                    myOpMode.telemetry.addData("tx", result.getTx());

                    // Access fiducial results
                    fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        distance = -fr.getCameraPoseTargetSpace().getPosition().z;
                        myOpMode.telemetry.addData("Distance", distance);
                        //myOpMode.telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }

                } else {
                    myOpMode.telemetry.addData("Limelight", "No data available");
                }

                double turretError = abs(result.getTx());
                double turretPower = turretPID.calculate(0, result.getTx());
                myOpMode.telemetry.addData("turretPower", turretPower);

                if (turretError > 0.5) {
                        turret.setPower(-turretPower);
                }

            /*double turretPower = turretPID.calculate(0, turretEncoder.getVoltage() * 10);
            myOpMode.telemetry.addData("turretPower", turretPower);
            turret.setPower(turretPower);*/
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
        } else if(myOpMode.gamepad1.right_trigger > 0.5){
            hoodMode = HoodMode.TUNING;
        }

        if(myOpMode.gamepad1.dpad_up){
            TUNINGRPM += 25;
        }else if(myOpMode.gamepad1.dpad_down){
            TUNINGRPM -= 25;
        }
        if(myOpMode.gamepad1.dpad_left){
            hoodTuning += 0.02;
        }else if(myOpMode.gamepad1.dpad_right){
            hoodTuning -= 0.02;
        }

        if(myOpMode.gamepad1.left_bumper) {
            turretMode = TurretMode.MANUAL;
        }
        if(myOpMode.gamepad1.right_bumper){
            turretMode = TurretMode.AUTO;
            hoodMode = HoodMode.AUTO;
        }

        measuredRPM = spin1.getVelocity() * 60 / TICKS_PER_REVOLUTION;
        double measuredRPM2 = spin2.getVelocity() * 60 / TICKS_PER_REVOLUTION;
        myOpMode.telemetry.addData("turret mode", turretMode);
        //myOpMode.telemetry.addData("turret pos", turret.getDirection().ordinal());
        //myOpMode.telemetry.addData("turret encoder", turretEncoder.getVoltage());
        //myOpMode.telemetry.addData("spin1Velocity", spin1.getVelocity());
        myOpMode.telemetry.addData("hood mode", hoodMode);
        myOpMode.telemetry.addData("targetRPM", revolutions_per_minute);
        myOpMode.telemetry.addData("measuredRPM", measuredRPM);
        //myOpMode.telemetry.addData("measuredRPM2", measuredRPM2);
        myOpMode.telemetry.addData("hoodPosition", hood.getPosition());
    }

    public Action launcherOn(String loc) {
        return new Action() {
            private boolean initialized = false;
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    TICKS_PER_SECOND = ((loc.equals("CLOSE") ? AUTO_CLOSE_RPM : AUTO_FAR_RPM) / 60) * TICKS_PER_REVOLUTION;
                    spin1.setVelocity(TICKS_PER_SECOND);
                    spin2.setVelocity(TICKS_PER_SECOND);
                    initialized = true;
                    timer.reset();
                }

                double measuredRPM = spin1.getVelocity() * 60 / TICKS_PER_REVOLUTION;
                myOpMode.telemetry.addData("measuredRPM2", measuredRPM);
                myOpMode.telemetry.addData("ticks per second", spin1.getVelocity());

                myOpMode.telemetry.update();

                return spin1.getVelocity() < TICKS_PER_SECOND && timer.seconds() < 2.5;
            }
        };
    }

    public Action launcherOff() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    spin1.setVelocity(0);
                    spin2.setVelocity(0);
                    initialized = true;
                }
                return spin1.getVelocity() > 0;
            }
        };
    }

    public Action autoAim() {
        return new Action() {
            private boolean initialized = false;
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                result = limelight.getLatestResult();
                if (result.isValid()) {

                /*
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();

                myOpMode.telemetry.addData("txnc", result.getTxNC());
                myOpMode.telemetry.addData("ty", result.getTy());
                myOpMode.telemetry.addData("tync", result.getTyNC());

                myOpMode.telemetry.addData("Botpose", botpose.toString());
                */

                    myOpMode.telemetry.addData("tx", result.getTx());

                    // Access fiducial results
                    fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        distance = -fr.getCameraPoseTargetSpace().getPosition().z;
                        myOpMode.telemetry.addData("Distance", distance);
                        //myOpMode.telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }

                } else {
                    myOpMode.telemetry.addData("Limelight", "No data available");
                }

                double turretError = abs(result.getTx());
                double turretPower = turretPID.calculate(0, result.getTx());
                myOpMode.telemetry.addData("turretPower", turretPower);

                if (turretError > 0.3) {
                    turret.setPower(-turretPower);
                }

                return turretError > 0.3 && timer.seconds() < 1;
            }
        };
    }

    public Action switchRed() {
        return new Action() {
            private boolean initialized = false;

            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                limelight.pipelineSwitch(4);

                return timer.seconds() < 0.2;
            }
        };
    }

    public Action switchBlue() {
        return new Action() {
            private boolean initialized = false;

            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                limelight.pipelineSwitch(2);

                return timer.seconds() < 0.2;
            }
        };
    }

    public Action scanMotif() {
        return new Action() {
            private boolean initialized = false;
            ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                LLResult result = limelight.getLatestResult();
                if (result.isValid()) {
                    // Access general information
                    // Access barcode results

                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        myOpMode.telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                        id = fr.getFiducialId();
                    }
                }

                myOpMode.telemetry.addData("id", id);
                myOpMode.telemetry.update();
                return timer.seconds() < 0.5;
            }
        };
    }



}

