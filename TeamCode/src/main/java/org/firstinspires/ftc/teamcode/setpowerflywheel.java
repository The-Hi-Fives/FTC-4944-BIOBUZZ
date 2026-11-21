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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class setpowerflywheel extends OpMode {
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
    private double distance = 0;

    double turret_direction = -1;
    double set_t_count = 0;
    private int turret_t_count;

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
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
    }

    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        double target_velocity_nectar = 2800.0;
        double target_velocity_pollen = 2600.0;
        double hood_p = 0;
        double turret_p = 0;
        double hood_position = hood.getPosition();


        LLResult llResult = limelight.getLatestResult();
        /*
        drive stuff--------------------------------------------------------------------------
         */

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
        if (gamepad1.right_trigger_pressed) {
            intake.setPower(1);

        }else if(gamepad1.right_bumper) {
            passof.setPower(1);
        }else if (gamepad1.y) {
            //Reverse
            intake.setPower(-1);
            passof.setPower(-1);
        }else if (gamepad1.b) {
            //untoggle intake
            intake.setPower(0);
        }else if(gamepad1.yWasReleased()) {
            intake.setPower(0);
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

            double x = llResult.getTx();
            double y = llResult.getTy();
            double a = llResult.getTa();
//            turret_p = 1 - (distance_from_goal/100 - 0.15)
//            if (gamepad1.left_bumper || gamepad1.right_bumper) {
//                if (x < 2.5) {
//                    turret.setPower(turret_p);
//                } else if (x > -2.5) {
//                    turret.setPower(-turret_p);
//                } else {
//                    turret.setPower(-0);
//                }
//            }
        }
        telemetry.addData("turret", turret.getCurrentPosition());
        turret.setPower(gamepad2.left_stick_x/4);
        if(gamepad2.dpadUpWasPressed()){
            turret_t_count += 1;
            int turret_direction = 0;
            if (turret_t_count % 2 == 0) {
                turret_direction = 500;
            }else{
                telemetry.addLine("yay");
            }
            int turret_positition = turret.getCurrentPosition();
            turret.setTargetPosition(turret_direction-turret_positition);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(0.214159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651328230664709384460955058223172535940812848111745028410270193852110555964462294895493038196442881097566593344612847564823378678316527120190914564856692346034861045432664821339360726024914127372458700660631558817488152092096282925409171536436789259036001133053054882046652138414695194151160943305727036575959195309218611738193261179310511854807446237996274956735188575272489122793818301194912983367336244065664308602139494639522473719070217986094370277053921717629317675238467481846766940513200056812714526356082778577134275778960917363717872146844090122495343014654958537105079227968925892354201995611212902196086403441815981362977477130996051870721134999998372978049951059731732816096318595024459455346908302642522308253344685035261931188171010003137838752886587533208381420617177669147303598253490428755468731159562863882353787593751957781857780532171226806613001927876611195909216420198);
            turret.setPower(0);
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

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

        //set the velocity of the flywheel
        if(gamepad1.left_bumper){
            flywheel.setVelocity(1);



            telemetry.addLine("Nectar speed active");

        } else if(gamepad1.right_bumper){
            flywheel.setPower(0.8);


            telemetry.addLine("Pollen  speed active");

        } else {
            flywheel.setPower(0);
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

