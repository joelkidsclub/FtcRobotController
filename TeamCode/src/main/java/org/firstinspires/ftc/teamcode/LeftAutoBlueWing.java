package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name="AAALeftAutoBlueWing", group = "drive")
//@Disabled

public class LeftAutoBlueWing extends LinearOpMode {
    /*
    elementPos for element position
       1 -> left
       2 -> mid
       3 -> right
    */
    int elementPos = 2; //Default to middle blue
    int targetTagBlue = 2;
    int targetTagRed = 2;

    int xValBackdrop = 2; //Default to middle blue
    //vars for object detection

    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "Blue_Cube.tflite";
    private static final String[] LABELS = {
            "Blue Element"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    boolean elementDetected = false;
    SampleMecanumDrive drive;

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    int step = 0;

    enum State {
        STATE_ZERO,
        STATE_INITIAL,
        STATE_STEP1_BACK25,
        STATE_CHECK_POS,
        STATE_POS1_STEP1,
        STATE_POS1_STEP2,
        STATE_POS1_STEP22,
        STATE_POS1_STEP23,
        STATE_POS1_STEP3,
        STATE_POS2_STEP1,
        STATE_POS2_STEP2,
        STATE_POS2_STEP3,
        STATE_POS3_STEP1,
        STATE_POS3_STEP2,
        STATE_POS3_STEP3,
        STATE_POS3_STEP4,
        STATE_POS3_STEP5,
        STATE_POS_REALIGN,
        STATE_PARK,//
        IDLE//
    }
    Trajectory traj_STATE_STEP1_BACK25;
    Trajectory traj_STATE_POS1_STEP1;
    Trajectory traj_STATE_POS1_STEP2;
    Trajectory traj_STATE_POS1_STEP22;
    Trajectory traj_STATE_POS1_STEP23;
    Trajectory traj_STATE_POS1_STEP3;
    Trajectory traj_STATE_POS2_STEP1;
    Trajectory traj_STATE_POS2_STEP2;
    Trajectory traj_STATE_POS2_STEP3;
    Trajectory traj_STATE_POS3_STEP1;
    Trajectory traj_STATE_POS3_STEP2;
    Trajectory traj_STATE_POS3_STEP3;


    LeftAutoBlueWing.State currentState = LeftAutoBlueWing.State.STATE_INITIAL;
    int ver = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  atDrive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        //Initialize
        initialize();
        elementPos = 1;

        if (elementPos == 1) {
            targetTagBlue = 1;
            targetTagRed = 4;
        }

        if (elementPos == 2) {
            targetTagBlue = 2;
            targetTagRed = 5;
        }

        if (elementPos == 3) {
            targetTagBlue = 3;
            targetTagRed = 6;
        }


        telemetry.addData("element position", elementPos);
        telemetry.update();

        currentState = LeftAutoBlueWing.State.STATE_INITIAL;

        if (isStopRequested()) return;


        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Current state 1=> ", currentState);

            switch (currentState) {
                case STATE_INITIAL:
                    step = 0;
                    telemetry.addData("STEP 0: STATE_INITIAL - drive.isBusy() ", drive.isBusy());
                    if (!drive.isBusy()) {
                        currentState = LeftAutoBlueWing.State.STATE_STEP1_BACK25;
                        //drive.followTrajectoryAsync(traj_Drive_To_Low_Junction);
                        telemetry.addData("STEP 1: STATE_INITIAL - ", "...");
                        //telemetry.update();
                    }
                    break;
                case STATE_STEP1_BACK25:
                    step = 1;
                    telemetry.addData("STEP 1: STATE_STEP1_BACK25: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        if (elementPos == 1)
                            currentState = LeftAutoBlueWing.State.STATE_POS1_STEP1;
                        else if (elementPos == 2)
                            currentState = LeftAutoBlueWing.State.STATE_POS2_STEP1;
                        else if (elementPos == 3)
                            currentState = LeftAutoBlueWing.State.STATE_POS3_STEP1;
                        else
                            currentState = LeftAutoBlueWing.State.STATE_POS2_STEP1;

                        telemetry.addData("STEP 1: STATE_STEP1_BACK25: nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_STEP1_BACK25);
                        telemetry.addData("STEP 1: traj_step1_back25 -","...");
                        //telemetry.update();
                    }
                    break;
                case STATE_POS1_STEP1:
                    step = 2;
                    telemetry.addData("STEP 2: STATE_POS1_STEP1: currentState => ", currentState);
                    currentState = LeftAutoBlueWing.State.STATE_POS1_STEP2;
                    telemetry.addData("STEP 2: STATE_POS1_STEP1: nextState => ", currentState);
                    //drive.followTrajectory(traj_STATE_POS1_STEP1);//lineToLinearHeading(new Pose2d(-25,-5, Math.toRadians(-90)))
                    drive.turn(Math.toRadians(-90));
                    break;
                case STATE_POS1_STEP2:
                    step =3;
                    telemetry.addData("STEP 3: STATE_POS1_STEP2: currentState => ", currentState);
                    currentState = State.STATE_POS1_STEP22;
                    telemetry.addData("STEP 3: STATE_POS1_STEP2: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS1_STEP2);//back(2)
                    //Drop pixel
                    break;
                case STATE_POS1_STEP22:
                    step =3;
                    telemetry.addData("STEP 3: STATE_POS1_STEP22: currentState => ", currentState);
                    currentState = LeftAutoBlueWing.State.STATE_POS1_STEP23;
                    telemetry.addData("STEP 3: STATE_POS1_STEP22: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS1_STEP22);//forward(5)
                    break;
                case STATE_POS1_STEP23:
                    step =3;
                    telemetry.addData("STEP 3: STATE_POS1_STEP22: currentState => ", currentState);
                    currentState = LeftAutoBlueWing.State.STATE_POS1_STEP3;
                    telemetry.addData("STEP 3: STATE_POS1_STEP22: nextState => ", currentState);
                    drive.turn(Math.toRadians(-180));
                    break;
                case STATE_POS1_STEP3:
                    step = 4;
                    telemetry.addData("STEP 4: STATE_POS1_STEP3: currentState => ", currentState);
                    currentState = LeftAutoBlueWing.State.STATE_POS_REALIGN;
                    telemetry.addData("STEP 4: STATE_POS1_STEP3: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS1_STEP3);//lineToLinearHeading(new Pose2d(-25,10, Math.toRadians(-180)))
                    break;
                case STATE_POS2_STEP1:
                    step = 5;
                    telemetry.addData("STEP 5: STATE_POS2_STEP1: currentState => ", currentState);
                    currentState = LeftAutoBlueWing.State.STATE_POS2_STEP2;
                    telemetry.addData("STEP 5: STATE_POS2_STEP1: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS2_STEP1);//back(5)
                    break;
                case STATE_POS2_STEP2:
                    step = 6;
                    telemetry.addData("STEP 6: STATE_POS2_STEP2: currentState => ", currentState);
                    currentState = LeftAutoBlueWing.State.STATE_POS2_STEP3;
                    telemetry.addData("STEP 6: STATE_POS2_STEP2: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS2_STEP2);//forward(5)
                    break;
                case STATE_POS2_STEP3:
                    step = 7;
                    telemetry.addData("STEP 7: STATE_POS2_STEP3: currentState => ", currentState);
                    currentState = LeftAutoBlueWing.State.STATE_POS_REALIGN;
                    telemetry.addData("STEP 7: STATE_POS2_STEP3: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS2_STEP3);//lineToLinearHeading(new Pose2d(-25,20, Math.toRadians(-90)))
                    break;
                case STATE_POS3_STEP1:
                    step = 8;
                    telemetry.addData("STEP 8: STATE_POS3_STEP1: currentState => ", currentState);
                    currentState = State.STATE_POS3_STEP2;
                    telemetry.addData("STEP 8: STATE_POS3_STEP1: nextState => ", currentState);
                    drive.turn(Math.toRadians(-90));
                    break;
                case STATE_POS3_STEP2:
                    step = 9;
                    telemetry.addData("STEP 9: STATE_POS3_STEP2: currentState => ", currentState);
                    currentState = State.STATE_POS3_STEP3;
                    telemetry.addData("STEP 9: STATE_POS3_STEP2: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS3_STEP1);
                    break;
                case STATE_POS3_STEP3:
                    step = 10;
                    telemetry.addData("STEP 10: STATE_POS3_STEP3: currentState => ", currentState);
                    currentState = LeftAutoBlueWing.State.STATE_POS3_STEP4;
                    telemetry.addData("STEP 10: STATE_POS3_STEP3: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS3_STEP2);
                    break;
                case STATE_POS3_STEP4:
                    step = 11;
                    telemetry.addData("STEP 11: STATE_POS3_STEP4: currentState => ", currentState);
                    currentState = State.STATE_POS3_STEP5;
                    telemetry.addData("STEP 11: STATE_POS3_STEP4: nextState => ", currentState);
                    drive.turn(Math.toRadians(180));
                    break;
                case STATE_POS3_STEP5:
                    telemetry.addData("STEP 12: STATE_POS3_STEP5: currentState => ", currentState);
                    currentState = LeftAutoBlueWing.State.STATE_POS_REALIGN;
                    telemetry.addData("STEP 12: STATE_POS3_STEP5: nextState => ", currentState);
                    //drive.turn(Math.toRadians(180));
                    drive.followTrajectory(traj_STATE_POS3_STEP3);
                    break;
                case STATE_POS_REALIGN:
                    step = 98;
                    telemetry.addData("STEP 98: STATE_POS_REALIGN: currentState => ", currentState);
                    currentState = LeftAutoBlueWing.State.STATE_PARK;
                    telemetry.addData("STEP 98: STATE_POS_REALIGN: nextState => ", currentState);
                    break;
                case STATE_PARK:
                    step = 99;
                    telemetry.addData("STEP 99: STATE_PARK: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = LeftAutoBlueWing.State.IDLE;
                        telemetry.addData("STEP 99: STATE_PARK", "...");
                    }
                    telemetry.addData("STEP 99: STATE_PARK: nextState => ", currentState);
                    break;
                case IDLE:
                    step = 100;
                    //Do Nothing
                    telemetry.addData("STEP 100: STATE_IDLE. Version =>", ver);
                    telemetry.update();
                    break;

            } //End switch
        } //End while
        telemetry.update();

    } //End runopmode

    private void initTfod() {

    }   // end method initTfod()


    public void driveToTag () {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  atDrive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }
        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (gamepad1.left_bumper && targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            atDrive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        } else {

            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
            atDrive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
            turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
            telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        telemetry.update();

        // Apply desired axes motions to the drivetrain.
        moveRobot(atDrive, strafe, turn);
        sleep(10);


    }
    public void initialize(){

        drive.setPoseEstimate(new Pose2d(0,0,0));

        long timeOut = (long) 0.1;

        //Trajectory definitions
        //Move back 25
        traj_STATE_STEP1_BACK25 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .back(25)
                .build();

        //Position 1
        traj_STATE_POS1_STEP1 = drive.trajectoryBuilder(traj_STATE_STEP1_BACK25.end())
                //.lineToLinearHeading(new Pose2d(-25,1, Math.toRadians(-45)))
                .forward(1)
                .build();
        //Drop
        traj_STATE_POS1_STEP2 = drive.trajectoryBuilder(traj_STATE_STEP1_BACK25.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .forward(18)
                .build();

        traj_STATE_POS1_STEP22 = drive.trajectoryBuilder(traj_STATE_POS1_STEP2.end())
                .forward(1)
                .build();

        traj_STATE_POS1_STEP3 = drive.trajectoryBuilder(traj_STATE_POS1_STEP22.end().plus(new Pose2d(0, 0, Math.toRadians(-180))))
                .back(1)
                .build();

        //Position 2
        traj_STATE_POS2_STEP1 = drive.trajectoryBuilder(traj_STATE_STEP1_BACK25.end())
                .back(5)
                .build();

        //Drop Pixel
        traj_STATE_POS2_STEP2 = drive.trajectoryBuilder(traj_STATE_POS2_STEP1.end())
                .forward(3)
                .build();

        traj_STATE_POS2_STEP3 = drive.trajectoryBuilder(traj_STATE_POS2_STEP2.end())
                .lineToLinearHeading(new Pose2d(-24,-20, Math.toRadians(90)))
                .build();

        //Position 3
        traj_STATE_POS3_STEP1 = drive.trajectoryBuilder(traj_STATE_STEP1_BACK25.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                //.lineToLinearHeading(new Pose2d(-25,10, Math.toRadians(-90)))
                .back(3)
                .build();

        traj_STATE_POS3_STEP2 = drive.trajectoryBuilder(traj_STATE_POS3_STEP1.end())
                //.lineToLinearHeading(new Pose2d(-25,10, Math.toRadians(-90)))
                .forward(3)
                .build();

        traj_STATE_POS3_STEP3 = drive.trajectoryBuilder(traj_STATE_POS3_STEP2.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                //.lineToLinearHeading(new Pose2d(-25,10, Math.toRadians(-90)))
                .back(20)
                .build();

        //Drop Pixel
        //traj_STATE_POS2_STEP2
        //traj_STATE_POS2_STEP3

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
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

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

        while(!isStarted() && !isStopRequested()){
            initTfod();
            tfod.setZoom(2.0);

            //sets element position depending on the position of the detected element
            //if object isn't detected, we are assuming it is element = 3 (default right)
            double x = 0;
            if (!tfod.getRecognitions().isEmpty()) {
                Recognition recognition = tfod.getRecognitions().get(0);
                x = (recognition.getLeft() + recognition.getRight()) / 2;

                //telemetry.addData("object detected:", x);
                if (x <= 250) {
                    elementPos = 1;
                } else if (x > 250) { //x coord for right){
                    elementPos = 2;
                }else{
                    elementPos = 3;
                }
            }
        } //End While
    } //End Init

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        //leftFrontDrive.setPower(leftFrontPower);
        //rightFrontDrive.setPower(rightFrontPower);
        //leftBackDrive.setPower(leftBackPower);
        //rightBackDrive.setPower(rightBackPower);
    }


} //End class
