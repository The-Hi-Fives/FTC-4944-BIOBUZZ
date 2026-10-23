package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class Mechanum2 extends OpMode {
    DcMotor fRight;
    DcMotor bRight;
    DcMotor fLeft;
    DcMotor bLeft;
    DcMotor intake;
    DcMotorEx flywheel;
    DcMotor passof;
    private Limelight3A limelight;
    private IMU imu;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        imu = hardwareMap.get(IMU.class, "pinpoint");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        fRight = hardwareMap.get(DcMotor.class, "fRight" );

        bRight = hardwareMap.get(DcMotor.class, "bRight" );

        fLeft = hardwareMap.get(DcMotor.class, "fLeft" );

        bLeft = hardwareMap.get(DcMotor.class, "bLeft" );

        //Declare drive motor directions
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");

        passof = hardwareMap.get(DcMotor.class, "passof");

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        // setup for driving
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        //adjusters for drive speed and a denominator to keep the motors at a constant speed of 1
        double drive_speed = gamepad1.left_trigger;
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);

        //driving actually
        fLeft.setPower(((forward + strafe + turn)*(1 - (drive_speed/2)))/denominator);
        bLeft.setPower(((forward - strafe + turn)*(1 - (drive_speed/2)))/denominator);
        bRight.setPower(((forward + strafe - turn)*(1 - (drive_speed/2)))/denominator);
        fRight.setPower(((forward - strafe - turn)*(1 - (drive_speed/2)))/denominator);

        //literally the intake doesn't need to be very complicated
        if (gamepad2.a) {
            intake.setPower(1);
        }else if (gamepad2.y) {
            //Reverse
            intake.setPower(-1);
            passof.setPower(-1);
        }else{
            passof.setPower(0);
            intake.setPower(0);
        }

        //declare the velocity speed as double so I can modify them easier and adjust them threw out the code
        double target_velocity_nectar = 1600.0;
        double target_velocity_pollen = 1300.0;

        if (gamepad2.dpad_up){
            //increase the speed by 50 velocity
            target_velocity_nectar += 50;
            target_velocity_pollen += 50;
        } else if(gamepad2.dpad_down){
            //decrease the speed by 50 velocity
            target_velocity_nectar -= 50;
            target_velocity_pollen -= 50;
        }

        telemetry.addData("TVN", target_velocity_nectar);
        telemetry.addData("TVP", target_velocity_pollen);

        // add speed of flywheel to tell it when to start the passof
        double speed;
        speed = flywheel.getVelocity();

        telemetry.addData("Flywheel Speed", speed);



        //add the yaw and the oreintation according to llight and imu
        YawPitchRollAngles oreintation = imu.getRobotYawPitchRollAngles();

        limelight.updateRobotOrientation(oreintation.getYaw());

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()){
            //add mecha tag two which updates 2d/3d field
            Pose3D botpose = llResult.getBotpose_MT2();
            //limelight telemetry
            telemetry.addLine("----------------------");
            telemetry.addData("TX,", llResult.getTx());
            telemetry.addData("TY,", llResult.getTy());
            telemetry.addData("TA,", llResult.getTa());
        }


        //set the velocity of the flywheel
        if(gamepad2.left_bumper){
            flywheel.setVelocity(target_velocity_nectar);

            // if the speed of the flywheel is greater than 100 less than the target_velocity_nectar
            if(speed > (target_velocity_nectar-150)){
                passof.setPower(1);
            }


            telemetry.addLine("Nectar speed active");

        } else if(gamepad2.right_bumper){
            flywheel.setVelocity(target_velocity_pollen);

            // if the speed of the flywheel is greater than 100 less than the target_velocity_pollen
            if(speed > (target_velocity_pollen-150)){
                passof.setPower(1);
            }

            telemetry.addLine("Pollen  speed active");

        } else {
            flywheel.setVelocity(0);
            passof.setPower(0);
        }

        //passof setup not very complicated added in other parts of code this is to run by self
        if(gamepad2.x){
            passof.setPower(1);
        }else{
            passof.setPower(0);
        }


        //telemetry.update so it updates telmetry so it knows what each value is
        telemetry.update();
    }
}

