package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;

public abstract class AutoCommon extends LinearOpMode {

    // motor configurations
    protected DcMotor         backleftDrive   = null;

    protected DcMotor         backrightDrive  = null;
    protected DcMotor         frontleftDrive   = null;

    protected DcMotor         frontrightDrive  = null;

    //protected DcMotor         rotator  = null;
    //linear slider
    protected DcMotor         slider  = null;
    public Servo grabber = null;
    public Servo tilt = null;
    public Servo extent = null;
    public Servo rotator = null;
    protected ElapsedTime     runtime = new ElapsedTime();

    // For motot encoders
    protected static final double     COUNTS_PER_MOTOR_REV    = 500 ;    // eg: TETRIX Motor Encoder
    protected static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    protected static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    protected static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    protected static final int RORATE_ARM_TICKS = 1500;
    protected static final double TILT_INIT_POSITION = 0.75;
    // IMU control
    public IMU imu;

    // Initial robot orientation
    public YawPitchRollAngles orientation0;
    public AngularVelocity angularVelocity0;
    public double yaw0;

    public double ticksPerInch = 31.3;
    public double ticksPerDegree = 15;
    // Tensor flow/april tag instance variables
    protected static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    //private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    //private static final String TFOD_MODEL_ASSET = "model_20230924.tflite";
    protected static final String TFOD_MODEL_ASSET = "model_cylinder2_20231125.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    protected static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
 /*
    private static final String[] LABELS = {
       "pixel yellow", "pixel white", "pixel green", "pixel purple", "Pixel"
    };
*/
    protected static final String[] LABELS = {
            "cylinder"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //protected TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    protected VisionPortal visionPortal;

    protected AprilTagProcessor aprilTag;

    // servo to place pixel on backboard



    public void initAll(){
        //initTfodAndAprilTag();
        initServo();
        initLinearSlider();
        initMotor();
        initIMU();
    }
    public void initServo() {
        grabber = hardwareMap.get(Servo.class, "grabber");
        grabber.setPosition(BotCoefficients.grabberClose);
        tilt = hardwareMap.servo.get("tilt");
        //tilt.setPosition(BotCoefficients.tiltUpInit);
        tilt.setPosition(0.47);
        extent = hardwareMap.servo.get("extent");
        extent.setPosition(0);
        rotator = hardwareMap.servo.get("rotator");
        rotator.setPosition(0.475);
        sleep(1000);
    }

    public void initLinearSlider() {
        slider = hardwareMap.dcMotor.get("slider");
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setPower(0);
    }
    public void extendSliderToHighBar() {
        slider.setTargetPosition(BotCoefficients.SLIDER_HIGH_BAR_HEIGHT);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(Math.abs(BotCoefficients.SLIDER_UP_SPEED));
    }

    public void hangSpeciman_old() {
        slider.setTargetPosition(BotCoefficients.SLIDER_HIGH_BAR_HEIGHT+400);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.1);
        sleep(900);
        grabber.setPosition(BotCoefficients.grabberSemiOpen);
        //tilt.setPosition(BotCoefficients.tiltDown);
    }

    public void hangSpeciman() {
        slider.setTargetPosition(BotCoefficients.SLIDER_HIGH_BAR_HEIGHT+1000);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.2);
        sleep(1200);
        //grabber.setPosition(BotCoefficients.grabberSemiOpen);
        //tilt.setPosition(BotCoefficients.tiltDown);
    }

    public void touchLowBar() {
        tilt.setPosition(BotCoefficients.tiltUp);
        extendSliderToHighBar();
        encoderDrive(0.2,  20,  20, 5.0);
        slider.setTargetPosition(BotCoefficients.SLIDER_HIGH_BAR_HEIGHT+650);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.2);
        sleep(900);
        //grabber.setPosition(BotCoefficients.grabberSemiOpen);
        //tilt.setPosition(BotCoefficients.tiltDown);
    }
public void driveAndHangSpeciman() {
    extendSliderToHighBar();
    sleep(1000);
    //drive to bars
    encoderDrive(0.2,  BotCoefficients.DISTANCE_TO_HIGH_BAR,  BotCoefficients.DISTANCE_TO_HIGH_BAR, 5.0);

    //extend linear slider to high bar
    //extendSliderToHighBar();
    //sleep(2000);
    //lower linear slider and open grabber
    hangSpeciman();
    sleep(1000);
    resetSliderGrabber();
}
    public void putSamplesInHighBacket() {

        //sleep(3000);
        // strafe to left

        rotator.setPosition(0.48);
        grabber.setPosition(BotCoefficients.grabberOpen);
        tilt.setPosition(BotCoefficients.tiltDown);
        extent.setPosition(BotCoefficients.EXTENT_LONG);

        sleep(1000);
        grabber.setPosition(BotCoefficients.grabberClose);
        sleep(1000);
        tilt.setPosition(BotCoefficients.tiltUp);

        slider.setTargetPosition(BotCoefficients.SLIDER_TOP_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.9);
        encoderDrive(0.3,  -41.5,  41.5, 5.0);
        extent.setPosition(0.92);
        sleep(1000);
        encoderDrive(0.2,  7,  7, 5.0);
        grabber.setPosition(BotCoefficients.grabberOpen);
        sleep(1000);
        encoderDrive(0.2,  -5,  -5, 5.0);
        extent.setPosition(BotCoefficients.EXTENT_SHORT);
        encoderDrive(0.3,  -4,  4, 5.0);

        slider.setTargetPosition(BotCoefficients.SLIDER_BOTTOM_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.7);

        encoderDrive(0.2,  -50,  -50, 5.0);
        //encoderDrive(0.3,  -10,  10, 5.0);


        sleep(2000);

    }

    public void putSamplesInHighBacket_2() {

        //sleep(3000);
        // strafe to left

        rotator.setPosition(0.48);
        grabber.setPosition(BotCoefficients.grabberOpen);
        tilt.setPosition(BotCoefficients.tiltDown);
        strafe_encoder(0.4, 52, 52, 5.0);
        extent.setPosition(0.82);
        sleep(1000);
        grabber.setPosition(BotCoefficients.grabberClose);
        sleep(1000);
        tilt.setPosition(0.53);

        slider.setTargetPosition(BotCoefficients.SLIDER_TOP_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.9);
        encoderDrive(0.3,  -41,  41, 5.0);
        extent.setPosition(0.92);
        sleep(1000);
        //encoderDrive(0.2,  6,  6, 5.0);
        grabber.setPosition(BotCoefficients.grabberOpen);
        sleep(1000);
        encoderDrive(0.3,  41,  -41, 5.0);
        strafe_encoder(0.3, 5, 5, 5.0);
        slider.setTargetPosition(BotCoefficients.SLIDER_BOTTOM_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.7);
        sleep(2000);

        //strafe_encoder(0.3, -5, -5, 5.0);
        tilt.setPosition(BotCoefficients.tiltDown);
        extent.setPosition(0.85);
        sleep(1000);
        grabber.setPosition(BotCoefficients.grabberClose);
        sleep(1000);
        tilt.setPosition(BotCoefficients.tiltUp);
        encoderDrive(0.3,  44,  -44, 5.0);
        slider.setTargetPosition(BotCoefficients.SLIDER_TOP_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.9);
        sleep(2000);
        encoderDrive(0.2,  6,  6, 5.0);
        grabber.setPosition(BotCoefficients.grabberOpen);
        sleep(1000);
    }
    public void pushSamples() {
        encoderDrive(0.3,  -10,  -10, 5.0);
        // strafe to left
        strafe_encoder(0.4, 35, 35, 5.0);
        encoderDrive(0.4,  35,  35, 5.0);


        // push first sample
        strafe_encoder(0.3, 11, 11, 5.0);
        encoderDrive(0.5,  -50,  -50, 5.0);
        encoderDrive(0.5,  50,  50, 5.0);

        // push second sample
        strafe_encoder(0.3, 15, 15, 5.0);
        encoderDrive(0.5,  -50,  -50, 5.0);
        encoderDrive(0.5,  50,  50, 5.0);

        // push third sample
        strafe_encoder(0.3, 15, 15, 5.0);
        encoderDrive(0.5,  -50,  -50, 5.0);
    }

    public void simpleParkLeft() {
        encoderDrive(0.2,  -25,  -25, 5.0);
        strafe_encoder(0.3, -85, -85, 5.0);
    }
    public void simpleParkRight() {
        encoderDrive(0.2,  -25,  -25, 5.0);
        // strafe to right
        strafe_encoder(0.4, -62, -62, 5.0);
    }

    public void pickSampleRight() {

        encoderDrive(0.2,  -10,  -10, 5.0);
        // strafe to right
        strafe_encoder(0.3, -47, -47, 5.0);
        tilt.setPosition(0.40);
        encoderDrive(0.3,  12,  12, 5.0);
        grabber.setPosition(BotCoefficients.grabberClose);
        sleep(1000);
        tilt.setPosition(BotCoefficients.tiltUp);
        encoderDrive(0.2,  -28,  -28, 5.0);
        turnToTargetYaw(90, 0.4, 3000);
        encoderDrive(0.5,  112,  112, 5.0);
        slider.setTargetPosition(BotCoefficients.SLIDER_TOP_POSITION);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.8);
        sleep(3000);
        grabber.setPosition(BotCoefficients.grabberOpen);
        sleep(1000);
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.8);
    }
    public void resetSliderGrabber() {
        //tilt.setPosition(BotCoefficients.tiltDown);
        grabber.setPosition(BotCoefficients.grabberOpen);
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.1);
    }
    public void initMotor(){
        // Initialize the drive system variables.
        backleftDrive  = hardwareMap.get(DcMotor.class, "bl");
        backrightDrive = hardwareMap.get(DcMotor.class, "br");
        frontleftDrive  = hardwareMap.get(DcMotor.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "fr");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        backrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);

        initLinearSlider();
    }
    /*
    public void initTfodAndAprilTag() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        builder.addProcessor(aprilTag);
        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()
*/
    public void strafe_encoder(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = backleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backrightDrive.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = frontleftDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontrightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            backleftDrive.setTargetPosition(newBackLeftTarget);
            backrightDrive.setTargetPosition(newBackRightTarget);
            frontleftDrive.setTargetPosition(newFrontLeftTarget);
            frontrightDrive.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backleftDrive.setPower(Math.abs(speed));
            backrightDrive.setPower(Math.abs(speed));
            frontleftDrive.setPower(Math.abs(speed));
            frontrightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (backleftDrive.isBusy() && backrightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        backleftDrive.getCurrentPosition(), backrightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            backleftDrive.setPower(0);
            backrightDrive.setPower(0);
            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = backleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backrightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = frontleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontrightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            backleftDrive.setTargetPosition(newBackLeftTarget);
            backrightDrive.setTargetPosition(newBackRightTarget);
            frontleftDrive.setTargetPosition(newFrontLeftTarget);
            frontrightDrive.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backleftDrive.setPower(Math.abs(speed));
            backrightDrive.setPower(Math.abs(speed));
            frontleftDrive.setPower(Math.abs(speed));
            frontrightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (backleftDrive.isBusy() && backrightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        backleftDrive.getCurrentPosition(), backrightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            backleftDrive.setPower(0);
            backrightDrive.setPower(0);
            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    /*
    public String detectTeamPropLine(String init_pos){
        String result = "right";
        for (int i = 0; i < 150; i++) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            if(currentRecognitions.isEmpty()){
                sleep(20);
            }
            else {
                telemetry.addData("# Objects Detected", currentRecognitions.size());

                // Step through the list of recognitions and display info for each one.
                //for (Recognition recognition : currentRecognitions) {
                Recognition recognition = currentRecognitions.get(0);
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

                if (x >350) {
                    result = "middle";
                    break;
                } else {
                    result = "left";
                    break;
                }
            }
            // end for() loop
        }
        return result;
    }
*/
    public void driveToBackBoardByAprilTag(int targetId) {
        //adjustment position according to apriltag
        double xPos = 10;
        double yPos = 10;
        int maxTries = 0;
        int idSeen = 0;
        int strafetime = 100;
        while (((xPos > 1) || (xPos < -1)) && (maxTries < 100)) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            idSeen = 0;
            // Step through the list of detections and display info for each one.

            for (AprilTagDetection detection : currentDetections) {

                idSeen = detection.id;
/*
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
*/
                if (detection.id == targetId) {
                    xPos = detection.ftcPose.x;
                    break;
                }
                else {
                    if (idSeen > 0) {
                        if (targetId < idSeen) {
                            xPos = -10;
                        } else {
                            xPos = 10;
                        }
                    }
                }

            }   // end for() loop

            if (idSeen > 0) {
                telemetry.addLine(String.format("Target found! xPos is %6.1f (inch)", xPos));
                telemetry.update();
                if (xPos > 6 || xPos < -6) {
                    strafetime = 700;
                }
                else {
                    strafetime = 100;
                }
                if (xPos > -2.0) {
                    strafe(0.3, strafetime);
                } else if (xPos < -4.0) {
                    strafe(-0.3, strafetime);
                } else {
                    break;
                }
            }
            sleep(20);
            maxTries++;
        }
    }

    public void strafe(double power, int milliseconds) {

        frontleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontleftDrive.setPower(power);
        backleftDrive.setPower(-power);
        frontrightDrive.setPower(-power);
        backrightDrive.setPower(power);
        sleep(milliseconds);
        //turnToTargetYaw2(90+yaw0, 0.4, 5000);
        frontleftDrive.setPower(0);
        backleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backrightDrive.setPower(0);

    }
    /*
    public void dropPixelOnLine() {
        grabberR.setPosition(BotCoefficients.grabberRightOpen);
        //grabberL.setPosition(0);
        //sleep(1000);
        //grabberTilt.setPosition(1);
        sleep(500);
        grabberTilt.setPosition(0.6);
        sleep(500);
        grabberR.setPosition(BotCoefficients.grabberRightClose);
        sleep(500);
        grabberTilt.setPosition(BotCoefficients.grabberUp);
        sleep(250);

    }

     */
    public void dropPixelOnBoard() {

    }

    public void forward(double power, int milliseconds) {
        frontleftDrive.setPower(power);
        frontrightDrive.setPower(power);
        backleftDrive.setPower(power);
        backrightDrive.setPower(power);

        sleep(milliseconds);
        stopRobot();
    }
    public void backward(double power, int milliseconds){
        frontleftDrive.setPower(-power);
        frontrightDrive.setPower(-power);
        backleftDrive.setPower(-power);
        backrightDrive.setPower(-power);

        sleep(milliseconds);
        stopRobot();
    }
    public void stopRobot(){
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
    }

    public void turn(double powerLeft, double powerRight, long milliseconds){
        frontleftDrive.setPower(powerLeft);
        frontrightDrive.setPower(powerRight);
        backleftDrive.setPower(powerLeft);
        backrightDrive.setPower(powerRight);
        sleep(milliseconds);
        stopRobot();
    }

    public void initIMU(){
        // Initialize IMU in the control hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        telemetry.addLine(String.format("INIT Yaw: %.1f\n", getCurrentYaw()));
        telemetry.update();

        // Retrieve the very initial Rotational Angles and Velocities
        orientation0 = imu.getRobotYawPitchRollAngles();
        angularVelocity0 = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        yaw0 = orientation0.getYaw(AngleUnit.DEGREES);

    }

    public double getCurrentYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    private void driveStrafe(int flTarget, int blTarget, int frTarget, int brTarget,
                             double power,
                             boolean bKeepYaw, double targetYaw){
        double currentYaw, diffYaw;
        double powerDeltaPct, powerL, powerR;
        int direction;

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setTargetPosition(flTarget);
        backleftDrive.setTargetPosition(blTarget);
        frontrightDrive.setTargetPosition(frTarget);
        backrightDrive.setTargetPosition(brTarget);

        frontleftDrive.setPower(power);
        backleftDrive.setPower(power);
        frontrightDrive.setPower(power);
        backrightDrive.setPower(power);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Defensive programming.
        // Use bKeepYaw only when all targets are the same, meaning moving in a straight line
        if (! ((flTarget == blTarget)
                && (flTarget == frTarget)
                && (flTarget == brTarget)) )
            bKeepYaw = false;
        direction = (flTarget > 0) ? 1 : -1;
        while(opModeIsActive() &&
                (frontleftDrive.isBusy() &&
                        backleftDrive.isBusy() &&
                        frontrightDrive.isBusy() &&
                        backrightDrive.isBusy())){
            if (bKeepYaw) {

                currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Math.abs(currentYaw - targetYaw) > 2.0)
                    powerDeltaPct = 0.25;
                else
                    powerDeltaPct = Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                if (currentYaw < targetYaw) {
                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);
                }
                else {
                    powerL = power * (1 + direction * powerDeltaPct);
                    powerR = power * (1 - direction * powerDeltaPct);
                }
                if (powerL > 1.0)
                    powerL = 1.0;
                if (powerR > 1.0)
                    powerR = 1.0;
                frontleftDrive.setPower(powerL);
                backleftDrive.setPower(powerL);
                frontrightDrive.setPower(powerR);
                backrightDrive.setPower(powerR);
            }
            idle();
        }

        frontleftDrive.setPower(0);
        backleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backrightDrive.setPower(0);
    }

    /*
    void RotateArm(int ticks, double power, long timeOutMills) {
        long timeCurrent, timeBegin;
        timeBegin = timeCurrent = System.currentTimeMillis();

        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(ticks);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(power);
        while(opModeIsActive()
                && rotator.isBusy()
                && (timeCurrent - timeBegin) < timeOutMills) {
            timeCurrent = System.currentTimeMillis();
        }
        rotator.setPower(0.1 * power);
    }

     */
    public void turnToTargetYaw3(double targetYawDegree, double power, long maxAllowedTimeInMills){
        long timeBegin, timeCurrent;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        int ticks, tickDirection;
        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);
        telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f", currentYaw, targetYawDegree));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diffYaw > 0.5
                && opModeIsActive()
                && ((timeCurrent-timeBegin) < maxAllowedTimeInMills)) {
            ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 130)
                ticks = 130;

            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 1)
                break;
            if (diffYaw > 3)
                factor = 1.0;
            else
                factor = diffYaw / 3;
            factor = 1.0;
            driveMotors(
                    (int)(tickDirection * ticks),
                    (int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    power * factor, false, 0);
            sleep(30);
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double)(timeCurrent-timeBegin)));
            telemetry.update();
        }
    }

    public void turnToTargetYaw2(double targetYawDegree, double power, long maxAllowedTimeInMills){
        long timeBegin, timeCurrent;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        int ticks, tickDirection;
        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);
        telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f", currentYaw, targetYawDegree));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diffYaw > 0.5
                && opModeIsActive()
                && ((timeCurrent-timeBegin) < maxAllowedTimeInMills)) {
            ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 600)
                ticks = 600;

            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 1) {
                break;
            }
            if (diffYaw > 3) {
                factor = 1.0;
            }
            else {
                factor = diffYaw / 3;
            }

                driveMotors(
                        (int) (tickDirection * ticks ),
                        (int) (tickDirection * ticks ),
                        -(int) (tickDirection * ticks),
                        -(int) (tickDirection * ticks),
                        power * factor, false, 0);

            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double)(timeCurrent-timeBegin)));
            telemetry.update();
        }
    }
    public void turnToTargetYaw4(double targetYawDegree, double power, long maxAllowedTimeInMills){
        long timeBegin, timeCurrent;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        int ticks, tickDirection;
        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);
        telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f", currentYaw, targetYawDegree));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diffYaw > 0.5
                && opModeIsActive()
                && ((timeCurrent-timeBegin) < maxAllowedTimeInMills)) {
            ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 600)
                ticks = 600;

            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 1)
                break;
            if (diffYaw > 3)
                factor = 1.0;
            else
                factor = diffYaw / 3;

                driveMotors(
                        (int) (tickDirection * ticks),
                        (int) (tickDirection * ticks),
                        -(int) (tickDirection * ticks),
                        -(int) (tickDirection * ticks),
                        power * factor, false, 0);

            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double)(timeCurrent-timeBegin)));
            telemetry.update();
        }
    }
    public void turnToTargetYaw(double targetYawDegree, double power, long maxAllowedTimeInMills){
        long timeBegin, timeCurrent;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        int ticks, tickDirection;
        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);
        telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f", currentYaw, targetYawDegree));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diffYaw > 0.5
                && opModeIsActive()
                && ((timeCurrent-timeBegin) < maxAllowedTimeInMills)) {
            ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 200)
                ticks = 200;

            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 1)
                break;
            if (diffYaw > 3)
                factor = 1.0;
            else
                factor = diffYaw / 3;
            driveMotors(
                    (int)(tickDirection * ticks),
                    (int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    power * factor, false, 0);
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double)(timeCurrent-timeBegin)));
            telemetry.update();
        }
    }
    private void driveMotors(int flTarget, int blTarget, int frTarget, int brTarget,
                             double power,
                             boolean bKeepYaw, double targetYaw){
        double currentYaw, diffYaw;
        double powerDeltaPct, powerL, powerR;
        double leftRatioToCounterCOG = 0.95;
        int direction;

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setTargetPosition(flTarget);
        backleftDrive.setTargetPosition(blTarget);
        frontrightDrive.setTargetPosition(frTarget);
        backrightDrive.setTargetPosition(brTarget);

        frontleftDrive.setPower(power * leftRatioToCounterCOG);
        backleftDrive.setPower(power * leftRatioToCounterCOG);
        frontrightDrive.setPower(power);
        backrightDrive.setPower(power);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Defensive programming.
        // Use bKeepYaw only when all targets are the same, meaning moving in a straight line
        if (! ((flTarget == blTarget)
                && (flTarget == frTarget)
                && (flTarget == brTarget)) )
            bKeepYaw = false;
        direction = (flTarget > 0) ? 1 : -1;
        while(opModeIsActive() &&
                (frontleftDrive.isBusy() &&
                        backleftDrive.isBusy() &&
                        frontrightDrive.isBusy() &&
                        backrightDrive.isBusy())){
            if (bKeepYaw) {

                currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Math.abs(currentYaw - targetYaw) > 2.0)
                    powerDeltaPct = 0.25;
                else
                    powerDeltaPct = Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                if (currentYaw < targetYaw) {
                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);
                }
                else {
                    powerL = power * (1 + direction * powerDeltaPct);
                    powerR = power * (1 - direction * powerDeltaPct);
                }
                if (powerL > 1.0)
                    powerL = 1.0;
                if (powerR > 1.0)
                    powerR = 1.0;
                frontleftDrive.setPower(powerL);
                backleftDrive.setPower(powerL);
                frontrightDrive.setPower(powerR);
                backrightDrive.setPower(powerR);
            }
            idle();
        }

        frontleftDrive.setPower(0);
        backleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backrightDrive.setPower(0);
    }
}
