package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;



import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class turret extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    DcMotorEx turret;
    DcMotorEx flywheel;
    DcMotor fRight;
    DcMotor bRight;
    DcMotor fLeft;
    DcMotor bLeft;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight = hardwareMap.get(DcMotor.class, "fRight" );

        bRight = hardwareMap.get(DcMotor.class, "bRight" );

        fLeft = hardwareMap.get(DcMotor.class, "fLeft" );

        bLeft = hardwareMap.get(DcMotor.class, "bLeft" );

        //Declare drive motor directions
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    }

    public void start(){
        limelight.start();
    }



    @Override
    public void loop() {

        double target_velocity_nectar = 1600.0;
        double target_velocity_pollen = 1300.0;

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

        YawPitchRollAngles oreintation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(oreintation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botpose = llResult.getBotpose_MT2();
            telemetry.addData("TX,", llResult.getTx());
            telemetry.addData("TY,", llResult.getTy());
            telemetry.addData("TA,", llResult.getTa());

            double x = llResult.getTx();
            double y = llResult.getTy();
            double a = llResult.getTa();
            if (gamepad1.left_bumper || gamepad1.right_bumper){
    //        while (x >= 1) {
    //            turret.setVelocity(400);
    //            x = llResult.getTx();
    //        }
    //        while(x <= -1) {
    //            turret.setVelocity(-400);
    //            x = llResult.getTx();
    //        }
    //        while(x>-1 && x<1){
    //            telemetry.addLine("Aligned with the goal!!!");
    //            turret.setVelocity(0);
    //            x = llResult.getTx();
    //        }

                if (x > 1) {
                    fRight.setPower(0.4);
                    fLeft.setPower(0.4);
                    bLeft.setPower(0.4);
                    bRight.setPower(0.4);

                } else if (x < -1) {

                    fRight.setPower(-0.4);
                    fLeft.setPower(-0.4);
                    bLeft.setPower(-0.4);
                    bRight.setPower(-0.4);

                } else if (x <= 1 && x > -1) {
                    fRight.setPower(-0);
                    fLeft.setPower(-0);
                    bLeft.setPower(-0);
                    bRight.setPower(-0);
                    telemetry.addLine("Aligned with the goal!!!");


                }
            }else{
                fRight.setPower(-0);
                fLeft.setPower(-0);
                bLeft.setPower(-0);
                bRight.setPower(-0);

            }

        }
        if (gamepad1.left_bumper) {
            flywheel.setVelocity(target_velocity_nectar);
        }else if (gamepad1.right_bumper){
            flywheel.setVelocity(target_velocity_pollen);
        }else{
            flywheel.setVelocity(0);
        }
        telemetry.addData("Velocity", flywheel.getVelocity());
        turret.setPower(gamepad2.right_stick_x);
        telemetry.addData("turret", turret.getCurrentPosition());


        telemetry.update();
    }
}
