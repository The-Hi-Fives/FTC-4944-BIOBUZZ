package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class Mechanum extends OpMode {
    DcMotor fRight;
    DcMotor bRight;
    DcMotor fLeft;
    DcMotor bLeft;
    DcMotor intake;
    DcMotorEx flywheel;
    DcMotor passof;
    private Limelight3A limelight;
    private IMU imu;

    private double Camera_Height = 14.73215;
    private double Camera_angle = 45;
    private double goal_height = 52.5;
    private double distance = 0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        fRight = hardwareMap.get(DcMotor.class, "fRight" );

        bRight = hardwareMap.get(DcMotor.class, "bRight" );

        fLeft = hardwareMap.get(DcMotor.class, "fLeft" );

        bLeft = hardwareMap.get(DcMotor.class, "bLeft" );

        //Declare drive motor directions
        fLeft.setDirection(DcMotor.Direction.REVERSE);

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
        /*
        drive stuff--------------------------------------------------------------------------
         */

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

        /*
        intake -------------------------------------------------------------------
        */
        //literally the intake doesn't need to be very complicated
        if (gamepad1.a) {
            intake.setPower(1);
        }else if (gamepad1.y) {
            //Reverse
            intake.setPower(-1);
            passof.setPower(-1);
        }else if (gamepad1.b) {
            //untoggle intake
            intake.setPower(0);
        }else{
            passof.setPower(0);
        }

        /*
        imu and limelight stuff -------------------------------------------
         */

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



        /*
        shooter stuff-------------------------------------------------------------
         */
        //declare the velocity speed as double so I can modify them easier and adjust them threw out the code
        double target_velocity_nectar = 1600.0;
        double target_velocity_pollen = 1300.0;

//        if (llResult != null && llResult.isValid()) {
//            double distance_from_goal = getDisatance(distance);
//            target_velocity_nectar = 1600.0 + (distance_from_goal * 2.0);
//            target_velocity_pollen = 1300.0 + (distance_from_goal * 1.8);
//        }

        telemetry.addData("TVN", target_velocity_nectar);
        telemetry.addData("TVP", target_velocity_pollen);


        //set the velocity of the flywheel
        if(gamepad1.left_bumper){
            flywheel.setVelocity(target_velocity_nectar);

            double speed = flywheel.getVelocity();
            // if the speed of the flywheel is greater than 100 less than the target_velocity_nectar
            if(speed > (target_velocity_nectar-150)){
                passof.setPower(1);
            }


            telemetry.addLine("Nectar speed active");

        } else if(gamepad1.right_bumper){
            flywheel.setVelocity(target_velocity_pollen);

            double speed = flywheel.getVelocity();
            // if the speed of the flywheel is greater than 100 less than the target_velocity_pollen
            if(speed > (target_velocity_pollen-150)){
                passof.setPower(1);
            }

            telemetry.addLine("Pollen  speed active");

        } else {
            flywheel.setVelocity(0);
            passof.setPower(0);
        }

        //pass off--------------------------------------------------------------------

        //pass off setup not very complicated added in other parts of code this is to run by self
        if(gamepad1.x){
            passof.setPower(1);
        }else{
            passof.setPower(0);
        }


        //telemetry.update so it updates telemetry so it knows what each value is
        telemetry.update();
    }

    public double getDisatance(double ty){
        double angleToTarget = Camera_angle + ty;
        double heightDifference = goal_height - Camera_Height;

        return heightDifference / Math.tan(Math.toRadians(angleToTarget));
    }


}

