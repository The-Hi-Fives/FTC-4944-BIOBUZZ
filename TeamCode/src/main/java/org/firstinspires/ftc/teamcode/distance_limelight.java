package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class distance_limelight extends OpMode {

    private DcMotorEx flywheel;
    private Limelight3A limelight;
    private Servo hood;
    private double Camera_Height = 14.73215;
    private double Camera_angle = 45;
    private double goal_height = 52.5;
    private double distance = 0;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);
    }

    public void start(){
        limelight.start();
    }
    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();

        if(llResult != null && llResult.isValid()){
            distance = getDisatance(llResult.getTy());
            telemetry.addData("Distance", distance);
        }else{
            telemetry.addData("No Valid Target", "Found");
        }

        double flywheel_speed = 0;
        telemetry.addData("distance", getDisatance(distance));
        double distance_from_goal = getDisatance(distance);


        if (gamepad1.left_bumper) {

            if(distance_from_goal<100){
                flywheel_speed = 1600;
                hood.setPosition(0.1);
            }
            if(distance_from_goal < 125 && distance_from_goal >= 100 ){
                flywheel_speed = 1700;
                hood.setPosition(0.2);
            }
            if(distance_from_goal < 150 && distance_from_goal >= 125 ){
                flywheel_speed = 1800;
                hood.setPosition(0.3);
            }
            if(distance_from_goal < 175 && distance_from_goal >= 150 ){
                flywheel_speed = 1900;
                hood.setPosition(0.4);
            }
            if(distance_from_goal < 200 && distance_from_goal >= 175 ){
                flywheel_speed = 2000;
                hood.setPosition(0.5);
            }
            if(distance_from_goal >= 200 && distance_from_goal < 225 ){
                flywheel_speed = 2100;
                hood.setPosition(0.6);
            }
            flywheel.setVelocity(flywheel_speed);
        }
        telemetry.update();
    }
    public double getDisatance(double ty){
        double angleToTarget = Camera_angle + ty;
        double heightDifference = goal_height - Camera_Height;

        return heightDifference / Math.tan(Math.toRadians(angleToTarget));
    }
}
