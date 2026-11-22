package org.firstinspires.ftc.teamcode;


import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous
public class auto_basic extends OpMode {
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
//
//        double targetTicksPerSec = 1600;
//        PIDFCoefficients pidf = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        flywheel.setVel

        turret = hardwareMap.get(DcMotorEx.class, " turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
        //250 == 1 foot


    }

    public void start(){
        hood.setPosition(0.067);
        limelight.start();
        drive_forward(250);
        flywheel.setVelocity(1800);
        sleep(2000);
        shoot();
        sleep(1500);
        shoot();
        sleep(1500);
        shoot();
        sleep(1500);
        shoot();
        passof.setPower(1);
        sleep(3000);
        flywheel.setVelocity(0);
        drive_forward(10*250);
        sleep(1000);
        strafe_right(1500);
        passof.setPower(0);
    }
    @Override
    public void loop() {
//        PIDFCoefficients pidfOrig = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        PIDFCoefficients pidfModified = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        telemetry.addData("P I D F (orig", "%.04f", "%.04f", "%.04f", "%0.4f", pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
//        telemetry.addData("PIDF (modified","%.04f", "%.04f", "%.04f", "%0.4f", pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
        telemetry.addData("Run time (sec)", getRuntime());



        LLResult llResult = limelight.getLatestResult();
        /*
        drive stuff--------------------------------------------------------------------------
         */





        double botHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        telemetry.addData("Heading", botHeading);

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
                        // Invert (-tx * KP) if your turret turns the wrong way
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
        }
        telemetry.addData("turret", turret.getCurrentPosition());



        /*
        shooter stuff-------------------------------------------------------------
         */
        //declare the velocity speed as double so I can modify them easier and adjust them threw out the code




        //Adding hood and shooter tuning to better soot the distance we are aiming from #need to tune!
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

        //set the velocity of the flywheel


        //pass off--------------------------------------------------------------------


        telemetry.update();
        }
        public void drive_forward ( long drivetdistance){
            fLeft.setPower(0.7);
            bLeft.setPower(0.7);
            fRight.setPower(0.7);
            bRight.setPower(0.7);
            sleep(drivetdistance);
            fLeft.setPower(0);
            bLeft.setPower(0);
            fRight.setPower(0);
            bRight.setPower(0);

        }
        public void shoot () {
            passof.setPower(0.55);
            intake.setPower(1);
            sleep(300);
            passof.setPower(0);
            intake.setPower(0);

        }
        public void strafe_right ( long drivedistance){
            fLeft.setPower(0.7);
            bLeft.setPower(-0.7);
            fRight.setPower(0.7);
            bRight.setPower(-0.7);
            sleep(drivedistance);
            fLeft.setPower(0);
            bLeft.setPower(0);
            fRight.setPower(0);
            bRight.setPower(0);

        }
        public void strafe_left ( long drivedistance){
            fLeft.setPower(-0.7);
            bLeft.setPower(0.7);
            fRight.setPower(-0.7);
            bRight.setPower(0.7);
            sleep(drivedistance);
            fLeft.setPower(0);
            bLeft.setPower(0);
            fRight.setPower(0);
            bRight.setPower(0);

        }
    public double getDisatance(double ty){
        double angleToTarget = Camera_angle + ty;
        double heightDifference = goal_height - Camera_Height;

        return heightDifference / Math.tan(Math.toRadians(angleToTarget));
    }



    }