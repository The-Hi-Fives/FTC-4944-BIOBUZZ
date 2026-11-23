package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Created by Tom on 9/26/17.  Updated 9/24/2021 for PIDF.
 * This assumes that you are using a REV Robotics Control Hub or REV Robotics Expansion Hub
 * as your DC motor controller.  This OpMode uses the extended/enhanced
 * PIDF-related functions of the DcMotorControllerEx class.
 */

@Autonomous(name="Concept: Change PIDF Controller", group = "Concept")
public class test extends LinearOpMode {

    // our DC motor
    DcMotor motorLeft;

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;
    public static final double NEW_F = 0.5;
    // These values are for illustration only; they must be set
    // and adjusted for each motor based on its planned usage.

    public void runOpMode() {
        // get reference to DC motor.
        motorLeft = hardwareMap.get(DcMotor.class, "flywheel");

        // wait for start command.
        waitForStart();

        // Get a reference to the motor controller and cast it as an extended functionality controller.
        // We assume it's a REV Robotics Control Hub or REV Robotics Expansion Hub, which supports the extended controller functions.
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)motorLeft.getController();

        // Get the port number of our configured motor.
        int motorIndex = ((DcMotorEx)motorLeft).getPortNumber();

        // Get the PIDF coefficients for the RUN_USING_ENCODER RunMode.
        PIDFCoefficients pidfOrig = motorControllerEx.getPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorControllerEx.setPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // Re-read coefficients and verify change.
        PIDFCoefficients pidfModified = motorControllerEx.getPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

        // Display info to user.
        while(opModeIsActive()) {
            telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                    pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
            telemetry.update();
        }
    }
}