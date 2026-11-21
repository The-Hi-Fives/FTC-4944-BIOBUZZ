package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Simple Turret Track", group = "Vision")
public class limelighttest extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor turret;

    // Tune these values on your robot
    private static final double KP = 0.02;         // Start small and increase until responsive without oscillating
    private static final double MAX_POWER = 0.5;     // Cap speed for safety
    private static final double DEADBAND_DEG = 1.0;  // Stop jittering when within 1 degree of center
    private static final int PIPELINE_INDEX = 0;     // Your active vision/AprilTag pipeline

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double turretPower = 0.0;

            if (result != null && result.isValid()) {
                double tx = result.getTx(); // Horizontal offset in degrees

                if (Math.abs(tx) > DEADBAND_DEG) {
                    // Invert sign (-tx * KP) if your turret turns the wrong way
                    turretPower = Range.clip(tx * KP, -MAX_POWER, MAX_POWER);
                }

                telemetry.addData("tx (deg)", "%.2f", tx);
            } else {
                telemetry.addData("Target", "None");
            }

            turret.setPower(turretPower);
            telemetry.addData("Turret Power", "%.2f", turretPower);
            telemetry.update();
        }

        limelight.stop();
    }
}
