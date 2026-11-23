package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Custom Flywheel PIDF")
public class pidf extends OpMode {

    private DcMotorEx flywheel;
    private ElapsedTime timer;

    public static double kP = 1.2;
    public static double kI = 0.117;
    public static double kD = 0.0000;
    public static double kF = 11.7; // Velocity feedforward (1 / max ticks per sec)

    private double lastError = 0;
    private double integralSum = 0;
    private double targetVelocity = 0;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        targetVelocity = gamepad1.right_trigger > 0.1 ? 1800.0 : 0.0;

        if (targetVelocity == 0) {
            flywheel.setPower(0);
            integralSum = 0;
            lastError = 0;
            return;
        }

        double currentVelocity = flywheel.getVelocity();
        double error = targetVelocity - currentVelocity;
        double dt = timer.seconds();
        timer.reset();

        integralSum += error * dt;
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative) + (kF * targetVelocity);
        flywheel.setPower(output);

        telemetry.addData("Target", targetVelocity);
        telemetry.addData("Actual", currentVelocity);
    }
}