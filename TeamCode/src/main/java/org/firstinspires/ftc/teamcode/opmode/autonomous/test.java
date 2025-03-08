/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes;
/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name=" TEST ", group="Robot")
@Disabled
public class test extends AutoCommon {

    @Override
    public void runOpMode() throws InterruptedException {

        //initAll();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        //waitForStart();
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            telemetry.addData("Drive Class", "Mecanum Drive");    //
            telemetry.update();
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            telemetry.addData("Drive Class", "Tank Drive");    //
            telemetry.update();
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
        sleep(5000);
        //sampleOnePlusThree();

    }



    private void sampleOnePlusTwo(){
        // Handle pre-loaded sample
        slider.setTargetPosition(BotCoefficients.SLIDER_TOP_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.95);

        encoderDrive(0.2,  5,  5, 5.0);
        strafe_encoder(0.4, 17, 17, 5.0);
        encoderDrive(0.4,  10,  -10, 5.0);

        tilt.setPosition(BotCoefficients.TILT_UP);
        sleep(1500);
        tilt.setPosition(BotCoefficients.TILT_DOWN);
        sleep(1000);

        slider.setTargetPosition(BotCoefficients.SLIDER_BOTTOM_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.9);
        //////////////////////////////////////////////////////////////
        // Get first sample
        extent.setPosition(0.36);
        sleep(1000);
        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_DOWN);

        encoderDrive(0.5,  -7,  7, 5.0);
        intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        encoderDrive(0.3,  5,  5, 5.0);

        //intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        sleep(500);

        intake.setPosition(BotCoefficients.INTAKE_INIT);

        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_INIT);
        extent.setPosition(BotCoefficients.EXTENT_BACK);
        sleep(1000);
        tilt.setPosition(BotCoefficients.TILT_INIT);
        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_UP);
        intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        sleep(1000);
        intake.setPosition(BotCoefficients.INTAKE_INIT);

        intake.setPosition(BotCoefficients.INTAKE_INIT);
        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_INIT);

        slider.setTargetPosition(BotCoefficients.SLIDER_TOP_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.95);

        encoderDrive(0.5,  7,  -7, 5.0);
        //strafe_encoder(0.5, -7, -7, 5.0);
        encoderDrive(0.5,  -5,  -5, 5.0);

        tilt.setPosition(BotCoefficients.TILT_UP);
        sleep(2000);
        tilt.setPosition(BotCoefficients.TILT_DOWN);
        sleep(500);

        slider.setTargetPosition(BotCoefficients.SLIDER_BOTTOM_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.8);
        /////////////////////////////////////////////////////////////////////
        //Get second sample
        extent.setPosition(0.36);
        //sleep(1000);
        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_DOWN);
        encoderDrive(0.5,  -11,  11, 5.0);
        intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        encoderDrive(0.3,  5,  5, 5.0);

        //intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        sleep(500);

        intake.setPosition(BotCoefficients.INTAKE_INIT);

        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_INIT);
        extent.setPosition(BotCoefficients.EXTENT_BACK);
        sleep(1000);
        tilt.setPosition(BotCoefficients.TILT_INIT);
        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_UP);
        intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        sleep(1000);
        intake.setPosition(BotCoefficients.INTAKE_INIT);

        intake.setPosition(BotCoefficients.INTAKE_INIT);
        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_INIT);

        slider.setTargetPosition(BotCoefficients.SLIDER_TOP_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.95);

        encoderDrive(0.5,  11,  -11, 5.0);
        encoderDrive(0.4,  -5,  -5, 5.0);

        tilt.setPosition(BotCoefficients.TILT_UP);
        sleep(2000);
        tilt.setPosition(BotCoefficients.TILT_DOWN);
        sleep(500);

        slider.setTargetPosition(BotCoefficients.SLIDER_BOTTOM_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.8);
        //encoderDrive(0.5,  12,  -12, 5.0);
        sleep(5000);

    }

    private void sampleOnePlusThree(){
        // Handle pre-loaded sample
        rotator.setPosition(0.42);
        slider.setTargetPosition(BotCoefficients.SLIDER_TOP_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.95);

        encoderDrive(0.4,  5,  5, 5.0);
        strafe_encoder(0.5, 17, 17, 5.0);

        tilt.setPosition(BotCoefficients.TILT_UP);
        encoderDrive(0.4,  10,  -10, 5.0);

        //tilt.setPosition(BotCoefficients.TILT_UP);
        sleep(500); //2000
        tilt.setPosition(BotCoefficients.TILT_DOWN);
        sleep(500); //1000

        slider.setTargetPosition(BotCoefficients.SLIDER_BOTTOM_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.9);
        //////////////////////////////////////////////////////////////
        // Get first sample
        extent.setPosition(0.42);
        //sleep(1000);
        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_DOWN);

        encoderDrive(0.5,  -7,  7, 5.0);
        intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        encoderDrive(0.3,  6,  6, 5.0);

        //intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        //sleep(200);

        intake.setPosition(BotCoefficients.INTAKE_INIT);

        rotator.setPosition(0.43);
        extent.setPosition(BotCoefficients.EXTENT_BACK);
        sleep(500);
        tilt.setPosition(BotCoefficients.TILT_INIT);
        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_UP);
        intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        sleep(700);
        intake.setPosition(BotCoefficients.INTAKE_INIT);

        //rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_INIT);

        rotator.setPosition(0.41);
        slider.setTargetPosition(BotCoefficients.SLIDER_TOP_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.95);

        tilt.setPosition(BotCoefficients.TILT_UP);

        encoderDrive(0.5,  7,  -7, 5.0);
        //strafe_encoder(0.5, -7, -7, 5.0);
        encoderDrive(0.5,  -5,  -5, 5.0);

        //tilt.setPosition(BotCoefficients.TILT_UP);
        //sleep(1500);
        tilt.setPosition(BotCoefficients.TILT_DOWN);
        sleep(500);

        slider.setTargetPosition(BotCoefficients.SLIDER_BOTTOM_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.8);

        /////////////////////////////////////////////////////////////////////
        //Get second sample
        extent.setPosition(0.51);
        //sleep(1000);
        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_DOWN);
        encoderDrive(0.5,  -11,  11, 5.0);
        intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        encoderDrive(0.3,  7,  7, 5.0);

        //intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        sleep(500);

        intake.setPosition(BotCoefficients.INTAKE_INIT);

        rotator.setPosition(0.43);
        extent.setPosition(BotCoefficients.EXTENT_BACK);
        sleep(500);
        tilt.setPosition(BotCoefficients.TILT_INIT);
        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_UP);
        intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        sleep(700);

        intake.setPosition(BotCoefficients.INTAKE_INIT);
        //rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_INIT);

        rotator.setPosition(0.41);
        slider.setTargetPosition(BotCoefficients.SLIDER_TOP_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.95);

        tilt.setPosition(BotCoefficients.TILT_UP);

        encoderDrive(0.5,  10,  -10, 5.0);
        encoderDrive(0.5,  -7,  -7, 5.0);

        //tilt.setPosition(BotCoefficients.TILT_UP);
        //sleep(1500);
        tilt.setPosition(BotCoefficients.TILT_DOWN);
        sleep(500);

        slider.setTargetPosition(BotCoefficients.SLIDER_BOTTOM_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.8);
        //encoderDrive(0.5,  12,  -12, 5.0);

        /////////////////////////////////////////////////////////////////////
        //Get third sample
        extent.setPosition(0.46);
        //sleep(1000);
        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_DOWN);
        encoderDrive(0.5,  -15,  15, 5.0);
        intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        encoderDrive(0.3,  5,  5, 5.0);

        //intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        sleep(200);

        intake.setPosition(BotCoefficients.INTAKE_INIT);

        rotator.setPosition(0.43);
        extent.setPosition(BotCoefficients.EXTENT_BACK);
        sleep(500);
        tilt.setPosition(BotCoefficients.TILT_INIT);
        rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_UP);
        intake.setPosition(BotCoefficients.INTAKE_FORWARD);
        sleep(700);

        intake.setPosition(BotCoefficients.INTAKE_INIT);
        //rotator.setPosition(BotCoefficients.INTAKE_ROTATOR_INIT);

        rotator.setPosition(0.41);
        slider.setTargetPosition(BotCoefficients.SLIDER_TOP_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.95);

        tilt.setPosition(BotCoefficients.TILT_UP);

        encoderDrive(0.5,  10,  -10, 5.0);
        encoderDrive(0.4,  -4,  -4, 5.0);

        //tilt.setPosition(BotCoefficients.TILT_UP);
        //sleep(1500);
        tilt.setPosition(BotCoefficients.TILT_DOWN);
        sleep(500);

        rotator.setPosition(0.41);
        slider.setTargetPosition(BotCoefficients.SLIDER_BOTTOM_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.8);

        sleep(5000);

    }
}
