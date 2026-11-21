package org.firstinspires.ftc.teamcode;


import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class auto_basic extends OpMode {
    DcMotor fRight;
    DcMotor bRight;
    DcMotor fLeft;
    DcMotor bLeft;
    DcMotor intake;
    DcMotorEx flywheel;
    DcMotorEx turret;
    DcMotor passof;
    Servo hood;

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        imu = hardwareMap.get(IMU.class, "imu");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();

        fRight = hardwareMap.get(DcMotor.class, "fRight" );
        bRight = hardwareMap.get(DcMotor.class, "bRight" );
        fLeft = hardwareMap.get(DcMotor.class, "fLeft" );
        bLeft = hardwareMap.get(DcMotor.class, "bLeft" );

        //Declare drive motor directions
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);

        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        passof = hardwareMap.get(DcMotor.class, "passof");
        passof.setDirection(DcMotorEx.Direction.REVERSE);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        double targetTicksPerSec = 1600;
        PIDFCoefficients pidf = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocity(targetTicksPerSec);

        turret = hardwareMap.get(DcMotorEx.class, " turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
        drive_forward();

    }

    public void start(){
        limelight.start();
    }
    @Override
    public void loop() {

    }
    public void drive_forward(){
        fLeft.setPower(0.7);
        bLeft.setPower(0.7);
        fRight.setPower(0.7);
        bRight.setPower(0.7);
        sleep(500);
        fLeft.setPower(0);
        bLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);

    }
    public void shoot(){
        passof.setPower(1);
        intake.setPower(1);
        sleep(500);
        passof.setPower(0);
        intake.setPower(0);
    }
    public void strafe(){
        fLeft.setPower(0.7);
        bLeft.setPower(-0.7);
        fRight.setPower(-0.7);
        bRight.setPower(0.7);
        sleep(500);
        fLeft.setPower(0);
        bLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);

    }
}
