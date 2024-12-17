package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PIDController;

@Config
@TeleOp
public class LiftPIDTest extends OpMode {

    public DcMotor rightLift = null;
    public DcMotor leftLift = null;

    PIDController leftLiftPID;
    PIDController rightLiftPID;

    public static double LIFT_KP = 0.01;
    public static double LIFT_KI = 0;
    public static double LIFT_KD = 0;
    public static final int LIFT_HIGH = 2300;
    public static final int LIFT_LOW = 1000;
    public static int LIFT_TARGET = 1300;

    public void init() {
        leftLiftPID = new PIDController(LIFT_KP, LIFT_KI, LIFT_KD);
        rightLiftPID = new PIDController(LIFT_KP, LIFT_KI, LIFT_KD);
        leftLiftPID.setMaxOut(0.95);
        rightLiftPID.setMaxOut(0.95);

        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData(">", "Lift Initialized");
    }

    public void loop() {
        double outLeft = leftLiftPID.calculate(LIFT_TARGET, leftLift.getCurrentPosition());
        double outRight = rightLiftPID.calculate(LIFT_TARGET, rightLift.getCurrentPosition());

        leftLift.setPower(outLeft);
        rightLift.setPower(outRight);

        telemetry.addData("LeftLift Power:", outLeft);
        telemetry.addData("RightLift Power:", outRight);

        telemetry.addData("LeftLift at:", leftLift.getCurrentPosition());
        telemetry.addData("RightLift at:", rightLift.getCurrentPosition());
        telemetry.addData("LiftTarget ", LIFT_TARGET);
        telemetry.update();
    }
}