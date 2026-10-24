package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class VelocityDrive extends OpMode {
    DcMotorEx fRight;
    DcMotorEx bRight;
    DcMotorEx fLeft;
    DcMotorEx bLeft;
    @Override
    public void init() {
        fRight = hardwareMap.get(DcMotorEx.class, "fRight" );

        bRight = hardwareMap.get(DcMotorEx.class, "bRight" );

        fLeft = hardwareMap.get(DcMotorEx.class, "fLeft" );

        bLeft = hardwareMap.get(DcMotorEx.class, "bLeft" );

        //Declare drive motor directions
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double power = (gamepad1.left_trigger);
        double flPower = ((forward + strafe + turn)*(1-(power/2.2)));
        double blPower = ((forward - strafe + turn)*(1-(power/2.2)));
        double frPower = ((forward - strafe - turn)*(1-(power/2.2)));
        double brPower = ((forward + strafe - turn)*(1-(power/2.2)));


        double max = Math.max(1.0, Math.max(Math.abs(flPower), Math.max(Math.abs(blPower), Math.max(Math.abs(frPower), Math.abs(brPower)))));
        flPower /= max;
        blPower /= max;
        frPower /= max;
        brPower /= max;

        final double MAX_TICKS_PER_SECOND = 2796.0; //max on a 312 motor!

        fLeft.setVelocity(flPower * MAX_TICKS_PER_SECOND);
        bLeft.setVelocity(blPower * MAX_TICKS_PER_SECOND);
        fRight.setVelocity(frPower * MAX_TICKS_PER_SECOND);
        bRight.setVelocity(brPower * MAX_TICKS_PER_SECOND);

        telemetry.addData("FR", fRight.getVelocity());
        telemetry.addData("BR", bRight.getVelocity());
        telemetry.addData("BL", bLeft.getVelocity());
        telemetry.addData("fL", fLeft.getVelocity());

        telemetry.update();

    }
}
