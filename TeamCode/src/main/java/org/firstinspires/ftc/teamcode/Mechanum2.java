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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class Mechanum2 extends OpMode {
    private static final double KP = 0.04;         // Start small and increase until responsive
    private static final double MAX_POWER = 0.4141592653589999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999;     // Cap speed for safety
    private static final double DEADBAND_DEG = 1.0;  // Stop jittering when within 1 degree of center

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

    private double Camera_Height = 14.73215;
    private double Camera_angle = 45;
    private double goal_height = 52.5;
    private double distance = 0.0;

    double turret_direction = -1;
    double set_t_count = 0;
    private int turret_t_count;

    private double hood_position_two = 0.04;
    public static final double P = 2.5;
    public static final double I = 0.1;
    public static final double D =0.2;
    public static final double F = 0.5;

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


        PIDFCoefficients pidfNew = new PIDFCoefficients(P,I,D,F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);



        turret = hardwareMap.get(DcMotorEx.class, " turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.FORWARD);

    }

    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        PIDFCoefficients pidfOrig = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfModified = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("P I D F (orig", "%.04f", "%.04f", "%.04f", "%0.4f", pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
        telemetry.addData("PIDF (modified","%.04f", "%.04f", "%.04f", "%0.4f", pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
        telemetry.addData("Run time (sec)", getRuntime());

        double target_velocity_nectar = 2300.0;
        double target_velocity_pollen = 2000.0;
        double hood_p = 0;
        double turret_p = 0;
       double hood_position = hood.getPosition();


        LLResult llResult = limelight.getLatestResult();
        /*
        drive stuff--------------------------------------------------------------------------
         */
        if (gamepad2.dpadLeftWasPressed()){
            hood_position_two += 0.04;
        }else if (gamepad2.dpadRightWasPressed()){
            hood_position_two -= 0.04;
        }else if (gamepad2.dpadUpWasPressed()){
            hood_position_two = 0.03;
        }
        hood_position_two = Range.clip(hood_position, 0.03, 0.35);

        if (gamepad1.dpadUpWasPressed()){
            pinpoint.resetPosAndIMU();
        }


        double botHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        telemetry.addData("Heading", botHeading);

        // setup for driving
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // start up the field centric's makes forward always forward!
        double strafe_ = strafe*Math.cos(botHeading) + forward*Math.sin(botHeading);
        double forward_ = forward*Math.cos(botHeading) - strafe*Math.sin(botHeading);

        //adjusters for drive speed and a denominator to keep the motors at a constant speed of 1
        double drive_speed = gamepad1.left_trigger;
        double denominator = Math.max(Math.abs(forward_) + Math.abs(strafe_) + Math.abs(turn), 1.0);

        //driving actually
        fLeft.setPower(((forward_ - strafe_ + turn)*(1 - (drive_speed/1.5)))/denominator);
        bLeft.setPower(((forward_ + strafe_ + turn)*(1 - (drive_speed/1.5)))/denominator);
        bRight.setPower(((forward_ + strafe_ - turn)*(1 - (drive_speed/1.5)))/denominator);
        fRight.setPower(((forward_ - strafe_ - turn)*(1 - (drive_speed/1.5)))/denominator);

        /*
        intake -------------------------------------------------------------------
        */

        //literally the intake doesn't need to be very complicated
        if (gamepad1.right_trigger_pressed || gamepad2.right_trigger_pressed) {
            intake.setPower(1);

        }else if (gamepad1.left_trigger_pressed || gamepad2.left_trigger_pressed) {
            //Reverse
            intake.setPower(-1);
            passof.setPower(-1);
        }else if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            //untoggle intake
            intake.setPower(0);
        }else if(gamepad1.leftTriggerWasReleased() || gamepad2.leftTriggerWasReleased()) {
            intake.setPower(0);
            passof.setPower(0);
        }else{
            passof.setPower(0);
        }

        /*
        imu and limelight stuff -------------------------------------------
         */




        if (llResult != null && llResult.isValid()){
            double distance_from_goal = getDisatance(distance);
            telemetry.addData("Distance from goal", distance_from_goal);
            //add mecha tag two which updates 2d/3d field
            Pose3D botpose = llResult.getBotpose_MT2();
            //limelight telemetry
            telemetry.addLine("----------------------");
            telemetry.addData("TX,", llResult.getTx());
            telemetry.addData("TY,", llResult.getTy());
            telemetry.addData("TA,", llResult.getTa());

            double tx = llResult.getTx();
            double y = llResult.getTy();
            double a = llResult.getTa();
            if (! gamepad2.dpad_down) {
                LLResult result = limelight.getLatestResult();
                double turretPower = 0.0;

                    if (result != null && result.isValid()) {


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
                    telemetry.addLine(":):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):):)");
                    telemetry.update();

            }
        }
        telemetry.addData("turret", turret.getCurrentPosition());
        turret.setPower(gamepad2.left_stick_x/4);


        /*
        shooter stuff-------------------------------------------------------------
         */
        //declare the velocity speed as double so I can modify them easier and adjust them threw out the code




        //Adding hood and shooter tuning to better soot the distance we are aming from #need  to tune!
//        if (llResult != null && llResult.isValid()) {
        //double deficit_N = distance_from_goal*2.0
        //double deficit_P = distance_from_goal*2.0
//
        // target_velocity_nectar += deficit_N
        // target_velocity_pollen += deficit_P
//
//       if (distance_from_goal >50){
//             hood_p = (distance_from_goal-50)/100
//             double hood_po = hood.get_Position
//             hood_p = Math.max(0.0, Math.min(1.0, hood_po));
//             hood.setPosisition(hood_p);
//        }

        telemetry.addData("TVN", target_velocity_nectar);
        telemetry.addData("TVP", target_velocity_pollen);
        telemetry.addData("flywheel", flywheel.getVelocity());
        //set the velocity of the flywheel
        if(gamepad2.left_bumper){
            flywheel.setVelocity(target_velocity_nectar);



            telemetry.addLine("Nectar speed active");

        } else if(gamepad2.right_bumper){
            flywheel.setVelocity(target_velocity_pollen);


            telemetry.addLine("Pollen  speed active");

        } else {
            flywheel.setPower(0);

        }

        //pass off--------------------------------------------------------------------

        //pass off setup not very complicated added in other parts of code this is to run by self
        if(gamepad1.x || gamepad2.x){
            passof.setPower(1);
        }else{
            passof.setPower(0);
        }
        hood.setPosition(hood_position);
        //telemetry.update so it updates telemetry so it knows what each value is
        telemetry.update();
    }

    public double getDisatance(double ty){
        double angleToTarget = Camera_angle + ty;
        double heightDifference = goal_height - Camera_Height;

        return heightDifference / Math.tan(Math.toRadians(angleToTarget));
    }

}