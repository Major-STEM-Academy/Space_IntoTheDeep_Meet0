package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
public class Teleop extends LinearOpMode {

    /*
    private enum State {
        ARM_UP,
        ARM_DOWN,
        HANG_INITIALIZATION,
        HANG,
    }
    State state = State.ARM_DOWN;
*/
    private ElapsedTime runtime = new ElapsedTime();
    Servo intake = null;
    Servo tilt = null;
    Servo extent = null;
    Servo rotator = null;
    Servo grabber = null;
    Servo grabber_tilt = null;
    TouchSensor touch = null;

    /*
    public static double GRABBER_INIT = 0.5;
    public static double GRABBER_FORWARD = 0;
    public static double GRABBER_BACKWARD = 1;

    public static double GRABBER_ROTATOR_INIT = 0.43;
    public static double GRABBER_ROTATOR_DOWN = 0.38;
    public static double GRABBER_ROTATOR_UP = 0.44;

    public static double EXTENT_INIT = 0.9;
    public static double EXTENT_OUT = 0.3;
    public static double EXTENT_BACK = EXTENT_INIT;

    public static double TILT_INIT = 0.285;
    public static double TILT_DOWN = TILT_INIT;
    public static double TILT_UP = 0.55;

     */
    private PIDController controller;

    public static double p = 0.004, i = 0, d = 0.00003;

    public static double f = 0.005;

    public static int target = 0;
    public boolean leftOpen = false;
    public boolean rightOpen = false;
    private final double ticks_in_degree = 700 / 180.0;

    private static final double STEP_INCHES = 1;
    private static final int TICKS_PER_INCH = 20;


    @Override
    public void runOpMode() throws InterruptedException {
        Motor frontLeft = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_312);

        DcMotor slider = hardwareMap.dcMotor.get("slider");
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setPower(0);

        DcMotor actuator = hardwareMap.dcMotor.get("actuator");
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        actuator.setPower(0);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("slider", "position (%d)", slider.getCurrentPosition());
        //telemetry.update();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("actuator", "position (%d)", actuator.getCurrentPosition());
        telemetry.update();

        intake = hardwareMap.servo.get("intake");
        intake.setPosition(BotCoefficients.INTAKE_INIT);

        tilt = hardwareMap.servo.get("tilt");
        //tilt.setPosition(BotCoefficients.tiltDown);
        //tilt.setPosition(0.26);
        tilt.setPosition(BotCoefficients.TILT_INIT);

        extent = hardwareMap.servo.get("extent");
        extent.setPosition(BotCoefficients.EXTENT_INIT);

        rotator = hardwareMap.servo.get("rotator");
        rotator.setPosition(0.4);

        grabber = hardwareMap.servo.get("grabber");
        grabber.setPosition(BotCoefficients.GRABBER_INIT);

        grabber_tilt = hardwareMap.servo.get("grabber_tilt");
        grabber_tilt.setPosition(BotCoefficients.GRABBER_TILT_INIT);

        touch = hardwareMap.get(TouchSensor.class, "touch_sensor");

        /*
        DcMotor lifter = hardwareMap.dcMotor.get("lifter");

        CRServo launchServo = hardwareMap.crservo.get("launcher");
        grabberTilt = hardwareMap.servo.get("grabberTilt");
        grabberR = hardwareMap.servo.get("grabberR");
        grabberL = hardwareMap.servo.get("grabberL");
        */

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        backRight.setInverted(true);
        frontRight.setInverted(true);

        controller = new PIDController(p, i, d);

        GamepadEx game1 = new GamepadEx(gamepad1);
        GamepadEx game2 = new GamepadEx(gamepad2);

        /*
        launchServo.setPower(0);
        grabberL.setPosition(BotCoefficients.grabberLeftClose);
        grabberR.setPosition(BotCoefficients.grabberRightClose);
        grabberTilt.setPosition(BotCoefficients.grabberUp);

         */

        waitForStart();


        while (!isStopRequested()) {

            double vertical = gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x*0.8;

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(turn), 1.2);
            double frontLeftPower = (vertical - horizontal + turn) / denominator;
            double frontRightPower = (vertical + horizontal - turn) / denominator;
            double backLeftPower = (vertical + horizontal + turn) / denominator;
            double backRightPower = (vertical - horizontal - turn) / denominator;

            frontLeft.set(frontLeftPower);
            frontRight.set(frontRightPower);
            backLeft.set(backLeftPower);
            backRight.set(backRightPower);

            // control slider
            // x button for down
            if (gamepad2.x){
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("slider", "position (%d)", slider.getCurrentPosition());
                telemetry.update();
                // down
                //int newSliderTarget = slider.getCurrentPosition() + (int)(STEP_INCHES * TICKS_PER_INCH);
                int newSliderTarget = slider.getCurrentPosition() + 150;
                if (newSliderTarget <= BotCoefficients.SLIDER_BOTTOM_POSITION) {
                    slider.setTargetPosition(newSliderTarget);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(Math.abs(BotCoefficients.SLIDER_DOWN_SPEED));
                }

                //slider.setPower(-1.0);
            }
            // y button for up
            else if(gamepad2.y){
                //int newSliderTarget = -(slider.getCurrentPosition() + (int)(STEP_INCHES * TICKS_PER_INCH));
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("slider", "position (%d)", slider.getCurrentPosition());
                telemetry.update();
                int newSliderTarget = slider.getCurrentPosition() - 120;
                if (newSliderTarget > -3000) {
                    //extent.setPosition(0.66);
                    slider.setTargetPosition(newSliderTarget);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(Math.abs(BotCoefficients.SLIDER_UP_SPEED));
                }
                //sleep(3000);
                //while(slider.isBusy() && opModeIsActive()) {
                    //Loop body can be empty
                //}
                //slider.setPower(0);
            }
            else {
                if ((slider.getCurrentPosition() > -2) || (touch.isPressed())){
                    slider.setPower(0);
                }
                else {
                    slider.setPower(BotCoefficients.SLIDER_HOLD_POWER);
                }
            }

            // up slider to low basket and high bar level
            if (gamepad1.y) {

                    actuator.setTargetPosition(BotCoefficients.ACTUATOR_TOP);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(Math.abs(0.8));
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("actuator", "position (%d)", actuator.getCurrentPosition());
                    telemetry.update();


            }
            else if (gamepad1.x) {

                    actuator.setTargetPosition(0);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(Math.abs(0.8));
                    telemetry.addData("Status", "Run Time: " + runtime.toString());
                    telemetry.addData("actuator", "position (%d)", actuator.getCurrentPosition());
                    telemetry.update();


            }
            else {
                int currentPos = actuator.getCurrentPosition();
                if ((currentPos <= 0) || (currentPos>=BotCoefficients.ACTUATOR_TOP)) {
                    actuator.setPower(0);
                }

            }
            // control horizontal extention
            if (gamepad2.a) {
                extent.setPosition(BotCoefficients.EXTENT_BACK);
            }

            if (gamepad2.b) {
                extent.setPosition(BotCoefficients.EXTENT_OUT);
            }

            // for testing purpose
            /*
            if (gamepad1.a) {
                extent.setPosition(0.5);
            }

            if (gamepad1.b) {
                extent.setPosition(0.9);
            }
            */

            // control grabber
            if (gamepad1.right_bumper) {
                // open
                grabber.setPosition(BotCoefficients.GRABBER_OPEN);
                //sleep(1500);
                //rotator.setPosition(0.42);
                //leftOpen = true;
            }
            if (gamepad1.right_trigger > 0.3){
                //close
                grabber.setPosition(BotCoefficients.GRABBER_CLOSE);
                //leftOpen = false;
            }
            // control grabber tilt
            if (gamepad1.left_bumper) {
                // open
                grabber_tilt.setPosition(BotCoefficients.GRABBER_TILT_UP);
                //sleep(1500);
                //rotator.setPosition(0.42);
                //leftOpen = true;
            }
            if (gamepad1.left_trigger > 0.3){
                //close
                grabber_tilt.setPosition(BotCoefficients.GRABBER_TILT_DOWN);
                //leftOpen = false;
            }
            // control intake tilt
            if (gamepad2.left_bumper) {
                // open
                if (extent.getPosition() > 0.85) {
                    rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_UP);
                }
                else {
                    rotator.setPosition(0.4);
                }
                //sleep(1500);
                //rotator.setPosition(0.42);
                //leftOpen = true;
            }
            if (gamepad2.left_trigger > 0.3){
                //close
                if (extent.getPosition() < 0.5) {
                    rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_DOWN_EXTENT);
                }
                else {
                    rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_DOWN);
                }
                //leftOpen = false;
            }

            //control vertical bucket tilt
            if ( gamepad2.right_bumper) {
                // open
                //tilt.setPosition(BotCoefficients.tiltUp);
                tilt.setPosition(BotCoefficients.TILT_UP);
            }
            if  (gamepad2.right_trigger > 0.3){
                //close
                //tilt.setPosition(BotCoefficients.tiltDown);
                tilt.setPosition(BotCoefficients.TILT_DOWN);
            }

            // for testing purpose
            /*
            if (gamepad1.right_bumper) {
                // open
                //tilt.setPosition(BotCoefficients.tiltUp);
                tilt.setPosition(0.58);
            }
            if (gamepad1.right_trigger > 0.3){
                //close
                //tilt.setPosition(BotCoefficients.tiltDown);
                tilt.setPosition(0.37);
            }

             */
            //control grabber
            if (gamepad2.dpad_up) {
                intake.setPosition(BotCoefficients.INTAKE_FORWARD);
            }
            if (gamepad2.dpad_down) {
                intake.setPosition(BotCoefficients.INTAKE_BACKWARD);
            }
            if (gamepad2.dpad_right || gamepad2.dpad_left ) {
                intake.setPosition(BotCoefficients.INTAKE_INIT);
            }
        }

    }






}