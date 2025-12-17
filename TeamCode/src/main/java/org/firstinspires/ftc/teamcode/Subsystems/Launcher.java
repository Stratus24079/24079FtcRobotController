package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utility.PIDController;

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
    public DcMotorEx spin = null;
    public Servo hood = null;
    public CRServo turret = null;
    public PIDController turretPID = null;

    public double FAR = 0.7;
    public double CLOSE = 1;
    public double FARRPM = 5500;
    public double CLOSERPM = 3500;
    public double revolutions_per_minute = 5000; //og: 5000
    public static final double TICKS_PER_REVOLUTION = 28;
    double TICKS_PER_SECOND = revolutions_per_minute / 60 * TICKS_PER_REVOLUTION;
    public double turretKP = 0.01;
    public double turretKI = 0;
    public double turretKD = 0;

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

    public enum TurretMode{
        MANUAL,
        AUTO,
    }
    public LauncherMode launcherMode = LauncherMode.OFF;
    public HoodMode hoodMode = HoodMode.CLOSE;
    public TurretMode turretMode = TurretMode.MANUAL;

    public Launcher (OpMode opmode){
        myOpMode = opmode;
    }

    public void init(){
        spin = myOpMode.hardwareMap.get(DcMotorEx.class, "launcher");
        spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spin.setDirection(DcMotor.Direction.REVERSE);

        hood = myOpMode.hardwareMap.get(Servo.class, "hood");
        hood.setPosition(CLOSE);

        turret = myOpMode.hardwareMap.get(CRServo.class, "turret");
        turret.setPower(0);

        turretPID = new PIDController(turretKP, turretKI, turretKD, 0.8);

        limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
    }

    public void update(){
        TICKS_PER_SECOND = (revolutions_per_minute / 60) * TICKS_PER_REVOLUTION;

        if (launcherMode == LauncherMode.ON) {
            spin.setVelocity(TICKS_PER_SECOND);
        } else if(launcherMode == LauncherMode.OFF) {
            spin.setVelocity(0);
        }

        if (hoodMode == HoodMode.FAR) {
            hood.setPosition(FAR);
            revolutions_per_minute = FARRPM;
        } else if(hoodMode == HoodMode.CLOSE) {
            hood.setPosition(CLOSE);
            revolutions_per_minute = CLOSERPM;
        }

        if(turretMode == TurretMode.MANUAL) {
            turret.setPower(myOpMode.gamepad2.right_stick_x);
        } else if (turretMode == TurretMode.AUTO) {


            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();

                myOpMode.telemetry.addData("tx", result.getTx());
                myOpMode.telemetry.addData("txnc", result.getTxNC());
                myOpMode.telemetry.addData("ty", result.getTy());
                myOpMode.telemetry.addData("tync", result.getTyNC());

                myOpMode.telemetry.addData("Botpose", botpose.toString());

                // Access barcode results
                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                for (LLResultTypes.BarcodeResult br : barcodeResults) {
                    myOpMode.telemetry.addData("Barcode", "Data: %s", br.getData());
                }

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    myOpMode.telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

            } else {
                myOpMode.telemetry.addData("Limelight", "No data available");
            }
            double turretError = Math.abs(result.getTx());
            double turretPower = turretPID.calculate(0, result.getTx());
            myOpMode.telemetry.addData("turretPower", turretPower);

            if (turretError > 3) {
                turret.setPower(-turretPower);
            }
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
        if(Math.abs(myOpMode.gamepad2.right_stick_x) > 0.5) {
            turretMode = TurretMode.MANUAL;
        }
        if(myOpMode.gamepad2.right_trigger > 0.5){
            turretMode = TurretMode.AUTO;
        }

        double measuredRPM = spin.getVelocity() * 60 / TICKS_PER_REVOLUTION;
        myOpMode.telemetry.addData("turret mode", turretMode);
        myOpMode.telemetry.addData("spinVelocity", spin.getVelocity());
        myOpMode.telemetry.addData("hood mode", hoodMode);
        myOpMode.telemetry.addData("targetRPM", revolutions_per_minute);
        myOpMode.telemetry.addData("measuredRPM", measuredRPM);


    }

    public Action launcherOn() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    spin.setPower(0.8);
                    initialized = true;
                }
                packet.put("ticks per second", spin.getCurrentPosition());
                return spin.getVelocity() < TICKS_PER_SECOND;
            }
        };
    }

    public Action launcherOff() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    spin.setPower(0);
                    initialized = true;
                }
                return spin.getVelocity() > 0;
            }
        };
    }
}
