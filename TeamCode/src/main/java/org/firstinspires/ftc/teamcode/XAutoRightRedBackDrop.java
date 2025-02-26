package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name="xAutoRightRedBackDrop", group = "drive")
@Disabled
public class XAutoRightRedBackDrop extends LinearOpMode {
    /*
    elementPos for element position
       1 -> left
       2 -> mid
       3 -> right
    */
    int elementPos = 2; //Default to middle blue
    int targetTagBlue = 2;
    int targetTagRed = 2;
    boolean targetFound = false;
    boolean elementDetected = false;
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    SampleMecanumDrive drive;
    private Servo pixelMover;
    //boolean pixelDropped = false;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "Red_Cube.tflite";
    private static final String[] LABELS = {
            "RedProp"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal myVisionPortal;

    int step = 0;

    enum State {
        STATE_ZERO,
        STATE_INITIAL,
        STATE_STEP1_BACK32,
        STATE_STEP1_BACK28,

        STATE_CHECK_POS,
        STATE_POS1_STEP1,
        STATE_POS1_STEP11,
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
        STATE_POS3_STEP6,
        STATE_POS3_STEP7,
        STATE_POS_REALIGN,
        STATE_PARK,//
        IDLE//
    }
    Trajectory traj_STATE_STEP1_BACK32;
    Trajectory traj_STATE_STEP1_BACK28;
    Trajectory traj_STATE_POS1_STEP1;
    Trajectory traj_STATE_POS1_STEP11;
    Trajectory traj_STATE_POS1_STEP2;
    Trajectory traj_STATE_POS1_STEP22;
    Trajectory traj_STATE_POS1_STEP23;
    Trajectory traj_STATE_POS1_STEP3;
    Trajectory traj_STATE_POS2_STEP1;
    Trajectory traj_STATE_POS2_STEP2;
    Trajectory traj_STATE_POS2_STEP3;
    Trajectory traj_STATE_POS3_STEP1;
    Trajectory traj_STATE_POS3_STEP2;
    Trajectory traj_STATE_POS3_STEP22;
    Trajectory traj_STATE_POS3_STEP3;
    Trajectory traj_STATE_POS3_STEP4;
    XAutoRightRedBackDrop.State currentState = XAutoRightRedBackDrop.State.STATE_INITIAL;
    int ver = 1;

    public int desiredTagId = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected


        initialize();
        elementPos = 3;
        //pixelDropped = false;

        if (elementPos == 1) {
            desiredTagId = 4;
        }

        if (elementPos == 2) {
            desiredTagId = 5;
        }

        if (elementPos == 3) {
            desiredTagId = 6;
        }

        telemetry.addData("Element position =>", elementPos);
        telemetry.addData("Desired tag =>", elementPos);
        telemetry.update();

        currentState = State.STATE_INITIAL;

        if (isStopRequested()) return;

        boolean done = false;

        while (!isStopRequested() && opModeIsActive() && !done) {
            telemetry.addData("Current state 1=> ", currentState);
            telemetry.update();
            switch (currentState) {
                case STATE_INITIAL:
                    step = 0;
                    telemetry.addData("STEP 0: STATE_INITIAL - drive.isBusy() ", drive.isBusy());
                    if (!drive.isBusy()) {
                        currentState = State.STATE_STEP1_BACK32;
                        //drive.followTrajectoryAsync(traj_Drive_To_Low_Junction);
                        telemetry.addData("STEP 1: STATE_INITIAL - ", "...");
                        telemetry.update();
                    }
                    visionPortal.setProcessorEnabled(tfod, true);
                    visionPortal.setProcessorEnabled(aprilTag, false);
                    break;
                case STATE_STEP1_BACK32:
                    step = 1;
                    telemetry.addData("STEP 1: STATE_STEP1_BACK32: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        if (elementPos == 1)
                            currentState = State.STATE_POS1_STEP1;
                        else if (elementPos == 2)
                            currentState = State.STATE_POS2_STEP1;
                        else if (elementPos == 3)
                            currentState = State.STATE_POS3_STEP1;
                        else
                            currentState = State.STATE_POS2_STEP1;

                    telemetry.addData("STEP 1: STATE_STEP1_BACK32: nextState => ", currentState);

                    if (elementPos == 1) {
                        drive.followTrajectory(traj_STATE_STEP1_BACK32);
                    }
                    else {
                        drive.followTrajectory(traj_STATE_STEP1_BACK28);
                    }
                    telemetry.addData("STEP 1: traj_step1_back32 -","...");
                    telemetry.update();
                    }
                    break;
                case STATE_POS1_STEP1:
                    step = 2;
                    telemetry.addData("STEP 2: STATE_POS1_STEP1: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS1_STEP11;
                        telemetry.addData("STEP 2: STATE_POS1_STEP1: nextState => ", currentState);
                    }
                    drive.turn(Math.toRadians(90));
                    telemetry.update();
                    break;
                case STATE_POS1_STEP11:
                    step = 3;
                    telemetry.addData("STEP 3: STATE_POS1_STEP1: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS1_STEP2;
                        telemetry.addData("STEP 3: STATE_POS1_STEP1: nextState => ", currentState);
                    }
                    drive.followTrajectory(traj_STATE_POS1_STEP1);//lineToLinearHeading(new Pose2d(-25,-5, Math.toRadians(-90)))
                    drive.followTrajectory(traj_STATE_POS1_STEP11);//lineToLinearHeading(new Pose2d(-25,-5, Math.toRadians(-90)))
                    telemetry.update();
                    break;
                case STATE_POS1_STEP2:
                    step =4;
                    pixelMover.setPosition(75.00);
                    sleep(1500);

                    telemetry.addData("STEP 4: STATE_POS1_STEP2: currentState => ", currentState);
                    currentState = State.STATE_POS_REALIGN;//STATE_POS1_STEP22;
                    telemetry.addData("STEP 4: STATE_POS1_STEP2: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS1_STEP2);//back(2)
                    telemetry.update();
                    break;
                case STATE_POS2_STEP1:
                    step = 5;
                    telemetry.addData("STEP 5: STATE_POS2_STEP1: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS2_STEP2;//STATE_POS2_STEP2
                        telemetry.addData("STEP 5: STATE_POS2_STEP1: nextState => ", currentState);
                    }
                    drive.followTrajectory(traj_STATE_POS2_STEP1);//back(5)
                    telemetry.update();
                    break;
                case STATE_POS2_STEP2:
                    step = 6;
                    telemetry.addData("STEP 6: STATE_POS2_STEP2: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS2_STEP3;
                        telemetry.addData("STEP 6: STATE_POS2_STEP2: nextState => ", currentState);
                    }

                    pixelMover.setPosition(75.00);
                    sleep(1500);
                    drive.followTrajectory(traj_STATE_POS2_STEP2);//forward(5)
                    telemetry.update();
                    break;
                case STATE_POS2_STEP3:
                    step = 7;
                    telemetry.addData("STEP 7: STATE_POS2_STEP3: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS_REALIGN;
                        telemetry.addData("STEP 7: STATE_POS2_STEP3: nextState => ", currentState);
                    }
                    drive.followTrajectory(traj_STATE_POS2_STEP3);//lineToLinearHeading(new Pose2d(-25,20, Math.toRadians(-90)))
                    telemetry.update();
                    break;
                case STATE_POS3_STEP1:
                    step = 8;
                    telemetry.addData("STEP 8: STATE_POS3_STEP1: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS3_STEP2;
                        telemetry.addData("STEP 8: STATE_POS3_STEP1: nextState => ", currentState);
                    }
                    drive.turn(Math.toRadians(-90));
                    //drive.followTrajectory(traj_STATE_POS3_STEP1);//lineToLinearHeading(new Pose2d(-25,10, Math.toRadians(-90)))
                    telemetry.update();
                    break;
                case STATE_POS3_STEP2:
                    step = 9;
                    telemetry.addData("STEP 9: STATE_POS3_STEP2: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS3_STEP3;
                        telemetry.addData("STEP 9: STATE_POS3_STEP2: nextState => ", currentState);
                    }
                    drive.followTrajectory(traj_STATE_POS3_STEP1);
                    drive.followTrajectory(traj_STATE_POS3_STEP2);
                    telemetry.update();
                    break;
                case STATE_POS3_STEP3:
                    step = 10;
                    telemetry.addData("STEP 10: STATE_POS3_STEP3: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS3_STEP4;
                        telemetry.addData("STEP 10: STATE_POS3_STEP3: nextState => ", currentState);
                    }
                    pixelMover.setPosition(75.00);
                    sleep(1500);
                    drive.followTrajectory(traj_STATE_POS3_STEP22);
                    telemetry.update();
                    break;
                case STATE_POS3_STEP4:
                    step = 11;
                    telemetry.addData("STEP 11: STATE_POS3_STEP4: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS3_STEP5;
                        telemetry.addData("STEP 11: STATE_POS3_STEP4: nextState => ", currentState);
                    }
                    //drive.turn(Math.toRadians(180));
                    telemetry.update();
                    break;
                case STATE_POS3_STEP5:
                    step = 12;
                    telemetry.addData("STEP 12: STATE_POS3_STEP5: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS3_STEP6;
                        telemetry.addData("STEP 12: STATE_POS3_STEP5: nextState => ", currentState);
                    }
                    //drive.turn(Math.toRadians(180));
                    drive.followTrajectory(traj_STATE_POS3_STEP3);
                    telemetry.update();
                    break;
                case STATE_POS3_STEP6:
                    step = 13;
                    telemetry.addData("STEP 13: STATE_POS3_STEP6: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS3_STEP7;
                        telemetry.addData("STEP 13: STATE_POS3_STEP6: nextState => ", currentState);
                    }
                    //drive.turn(Math.toRadians(180));
                    telemetry.update();
                    break;
                case STATE_POS3_STEP7:
                    step = 14;
                    telemetry.addData("STEP 14: STATE_POS3_STEP7: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS_REALIGN;
                        telemetry.addData("STEP 14: STATE_POS3_STEP7: nextState => ", currentState);
                    }
                    //drive.turn(Math.toRadians(180));
                    drive.followTrajectory(traj_STATE_POS3_STEP4);
                    telemetry.update();
                    break;
                case STATE_POS_REALIGN:
                    step = 98;
                    telemetry.addData("STEP 98: STATE_POS_REALIGN: currentState => ", currentState);
                    pixelMover.setPosition(0);
                    visionPortal.setProcessorEnabled(tfod, false);
                    visionPortal.setProcessorEnabled(aprilTag, true);
                    telemetry.update();
                    detectAprilTag();
                    telemetryAprilTag();

                    if (!drive.isBusy() && targetFound) {
                        currentState = State.STATE_PARK;
                        telemetry.addData("STEP 98: STATE_POS_REALIGN: nextState => ", currentState);
                        telemetry.update();
                    }

                    break;
                case STATE_PARK:
                    step = 99;
                    telemetry.addData("STEP 99: STATE_PARK: currentState => ", currentState);

                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                        telemetry.addData("STEP 99: STATE_PARK", "...");
                    }
                    telemetry.addData("STEP 99: STATE_PARK: nextState => ", currentState);
                    telemetry.update();
                    break;
                case IDLE:
                    step = 100;
                    //Do Nothing
                    done = true;
                    telemetry.addData("STEP 100: STATE_IDLE. Version =>", ver);
                    telemetry.update();
                    break;

            } //End switch
        } //End while
        telemetry.update();

    } //End runopmode

    private void initTfod() {

    }   // end method initTfod()

    public void initialize(){
        pixelMover = hardwareMap.get(Servo .class, "pixeldrop");

        pixelMover.setPosition(0.00);

        // Create the TensorFlow processor by using a builder.
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------
        aprilTag = new AprilTagProcessor.Builder()
                .build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------
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
        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }

        visionPortal.setProcessorEnabled(tfod, true);
        visionPortal.setProcessorEnabled(aprilTag, false);

        drive.setPoseEstimate(new Pose2d(0,0,0));

        long timeOut = (long) 0.1;

        //Trajectory definitions
        //Move back 25
        traj_STATE_STEP1_BACK32 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .back(32)
                .build();

        traj_STATE_STEP1_BACK28 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .back(28)
                .build();

        traj_STATE_POS1_STEP1 = drive.trajectoryBuilder(traj_STATE_STEP1_BACK28.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                //.lineToLinearHeading(new Pose2d(-25,1, Math.toRadians(-45)))
                .back(5)
                .build();

        traj_STATE_POS1_STEP11 = drive.trajectoryBuilder(traj_STATE_POS1_STEP1.end())
                .forward(2)
                .build();
        //Drop
        traj_STATE_POS1_STEP2 = drive.trajectoryBuilder(traj_STATE_POS1_STEP11.end())
                .lineToLinearHeading(new Pose2d(-38,42, Math.toRadians(-90)))//-27,30 orig; -25, 35 for 6
                //.back(1)
                .build();
/*
        traj_STATE_POS1_STEP22 = drive.trajectoryBuilder(traj_STATE_POS1_STEP2.end())
                .forward(6)
                .build();

        traj_STATE_POS1_STEP3 = drive.trajectoryBuilder(traj_STATE_POS1_STEP22.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                .back(20)
                .build();
*/
        //Position 2
        traj_STATE_POS2_STEP1 = drive.trajectoryBuilder(traj_STATE_STEP1_BACK32.end())
                .forward(5)
                .build();

        //Drop Pixel
        traj_STATE_POS2_STEP2 = drive.trajectoryBuilder(traj_STATE_POS2_STEP1.end())
                .forward(6)
                .build();

        traj_STATE_POS2_STEP3 = drive.trajectoryBuilder(traj_STATE_POS2_STEP2.end())
                .lineToLinearHeading(new Pose2d(-24,39.5, Math.toRadians(-90)))//-27,30 orig; -25, 35 for 6
                .build();

        //Position 3
        traj_STATE_POS3_STEP1 = drive.trajectoryBuilder(traj_STATE_STEP1_BACK28.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .back(8)
                .build();

        traj_STATE_POS3_STEP2 = drive.trajectoryBuilder(traj_STATE_POS3_STEP1.end())
                //.lineToLinearHeading(new Pose2d(-25,10, Math.toRadians(-90)))
                .forward(7)
                .build();

        traj_STATE_POS3_STEP22 = drive.trajectoryBuilder(traj_STATE_POS3_STEP2.end())
                //.lineToLinearHeading(new Pose2d(-25,10, Math.toRadians(-90)))
                .strafeLeft(15)
                .build();

        /*
        traj_STATE_POS3_STEP3 = drive.trajectoryBuilder(traj_STATE_POS3_STEP22.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                .lineToLinearHeading(new Pose2d(-18,39, Math.toRadians(-90)))
                .build();
        */

        traj_STATE_POS3_STEP3 = drive.trajectoryBuilder(traj_STATE_POS3_STEP22.end())
                .lineToLinearHeading(new Pose2d(-18,39, Math.toRadians(-90)))
                .build();

        traj_STATE_POS3_STEP4 = drive.trajectoryBuilder(traj_STATE_POS3_STEP3.end())
                .strafeLeft(1)//20
                .build();

        //Drop Pixel
        //traj_STATE_POS2_STEP2
        //traj_STATE_POS2_STEP3

        while(!isStarted() && !isStopRequested()){
            initTfod();
            tfod.setZoom(1.0);

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

    public void detectAprilTag(){
        targetFound = false;
        desiredTag  = null;
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);

        telemetry.addData("In Detect ATag. Looking for: ", desiredTagId);
        telemetry.update();

        sleep(1000);
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            sleep(1000);
            telemetry.addData("something detected. Id:", detection.id);
            telemetry.update();
            if (detection.metadata != null) {
                sleep(250);
                //  Check to see if we want to track towards this tag.
                if ((desiredTagId < 0) || (detection.id == desiredTagId)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    //telemetry.update();

                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                //telemetry.update();
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            sleep(500);
            telemetry.addData("id","matches");
            //telemetry.update();

        } else {
            sleep(500);
            telemetry.addData("\n>","find valid target\n");
            //telemetry.update();
        }

        telemetry.addData("Exit Detect ATag", "...");
        telemetry.update();
    }

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();
        }   // end for() loop

    }   // end method telemetryTfod()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.update();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            sleep(1000);
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
            telemetry.update();
        }   // end for() loop

    }   // end method telemetryAprilTag()


} //End class
