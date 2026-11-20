package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp(name = "Batman is cool unlike you")
public class Fc_test extends OpMode {
    DcMotor fRight;
    DcMotor bRight;
    DcMotor fLeft;
    DcMotor bLeft;
    DcMotor intake;
    DcMotorEx flywheel;
    DcMotor passof;
    private GoBildaPinpointDriver pinpoint;
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

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

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

        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        passof = hardwareMap.get(DcMotor.class, "passof");
        passof.setDirection(DcMotor.Direction.FORWARD);
        passof.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
    }

    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        if (gamepad1.dpadLeftWasPressed()){
            imu.resetYaw();
        }
        /*
        drive stuff--------------------------------------------------------------------------
         */
        Orientation robotOreintation;
        robotOreintation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // setup for driving
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        //field centric's
        double strafe_ = strafe*Math.cos(robotOreintation.thirdAngle) + forward*Math.sin(robotOreintation.thirdAngle);
        double forward_ = forward*Math.cos(robotOreintation.thirdAngle) - strafe*Math.sin(robotOreintation.thirdAngle);

        //adjusters for drive speed and a denominator to keep the motors at a constant speed of 1
        double drive_speed = gamepad1.left_trigger;
        double denominator = Math.max(Math.abs(forward_) + Math.abs(strafe_) + Math.abs(turn), 1.0);

        //driving actually
        fLeft.setPower(((forward_ + strafe_ + turn)*(1 - (drive_speed/2)))/denominator);
        bLeft.setPower(((forward_ - strafe_ + turn)*(1 - (drive_speed/2)))/denominator);
        bRight.setPower(((forward_ + strafe_ - turn)*(1 - (drive_speed/2)))/denominator);
        fRight.setPower(((forward_ - strafe_ - turn)*(1 - (drive_speed/2)))/denominator);

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
//            target_velocity_pollen = 1300.0 + (distance_from_goal * 2.0);
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

