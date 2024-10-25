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
    Servo grabber = null;
    //Servo grabberTilt = null;
    //Servo grabberR = null;
    //Servo grabberL = null;

    // pid

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

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("slider", "position (%d)", slider.getCurrentPosition());
        telemetry.update();

        grabber = hardwareMap.servo.get("grabber");
        grabber.setPosition(BotCoefficients.grabberClose);

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

            double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(turn), 1);
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
                // down
                //int newSliderTarget = slider.getCurrentPosition() + (int)(STEP_INCHES * TICKS_PER_INCH);
                int newSliderTarget = slider.getCurrentPosition() + 40;
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
                int newSliderTarget = slider.getCurrentPosition() - 80;
                if (newSliderTarget > BotCoefficients.SLIDER_TOP_POSITION) {
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
                if (slider.getCurrentPosition() > -3) {
                    slider.setPower(0);
                }
                else {
                    slider.setPower(BotCoefficients.SLIDER_HOLD_POWER);
                }
            }

            // up slider to low basket and high bar level
            if (gamepad1.y) {
                slider.setTargetPosition(BotCoefficients.SLIDER_LOW_BASKET_HEIGHT);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(Math.abs(BotCoefficients.SLIDER_UP_SPEED));
            }
            else if (gamepad1.x) {
                slider.setTargetPosition(0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(Math.abs(BotCoefficients.SLIDER_DOWN_SPEED));
            }
            // control grabber
            if (gamepad2.left_bumper || gamepad1.left_bumper) {
                // open
                grabber.setPosition(BotCoefficients.grabberOpen);
                leftOpen = true;
            }
            if ((gamepad2.left_trigger > 0.3) || (gamepad1.left_trigger > 0.3)){
                //close
                grabber.setPosition(BotCoefficients.grabberClose);
                leftOpen = false;
            }

        }

    }






}