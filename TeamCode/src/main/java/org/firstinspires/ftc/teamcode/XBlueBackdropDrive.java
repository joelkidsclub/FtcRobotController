package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
@Autonomous(name="XAutoBlueBackdropDrive", group = "drive")
@Disabled
public class XBlueBackdropDrive extends LinearOpMode {
    //april tag processor
    AprilTagProcessor aprilTag;
    VisionPortal myVisionPortal;
    Trajectory forwardPush1;
    Trajectory back2;

    //spines to the backdrop depending on element position (look at x val on trajectory in initTraj)
    Trajectory splineToBackdrop3;
    //strafes right to park right
    Trajectory strafeParkRight4;
    //goes forward to finish park (right)
    Trajectory forwardParkRight5;
    //strafes left to park left
    Trajectory strafeParkLeft4;
    //goes forward to finish park (left)
    Trajectory forwardParkLeft5;
    /*
   elementPos for element position
       1 -> left
       2 -> mid
       3 -> right
    */
    int elementPos = 3;
    int xValBackdrop = 1;

    //vars for object detection
    private static final boolean USE_WEBCAM = true;
//    private static final String TFOD_MODEL_ASSET = "Blue_Cube.tflite";
    //Red_Cube.tflite => RedProp
    //Blue_Cube.tflite => BlueProp
    private static final String TFOD_MODEL_ASSET = "Red_Cube.tflite";

    private static final String[] LABELS = {
            "RedProp"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    boolean elementDetected = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //method for arm and stuff

        //will run while the code hasn't been run
        initialize();
        myVisionPortal.setProcessorEnabled(tfod, true);
        telemetry.addData("element position", elementPos);
        telemetry.update();

        initTraj(drive, elementPos);


        if (isStopRequested()) return;


        drive.followTrajectory(splineToBackdrop3);

        myVisionPortal.setProcessorEnabled(tfod, false);
        myVisionPortal.setProcessorEnabled(aprilTag, true);

        detectAprilTag();

        sleep(5000);
        myVisionPortal.setProcessorEnabled(aprilTag, false);


        if (elementPos == 3) {
            drive.followTrajectory(strafeParkRight4);
            drive.followTrajectory(forwardParkRight5);
        } else {
            drive.followTrajectory(strafeParkLeft4);
            drive.followTrajectory(forwardParkLeft5);
        }


        Pose2d poseEstimate = drive.getPoseEstimate();
        //telemetry.addData("finalX", poseEstimate.getX());
        //telemetry.addData("finalY", poseEstimate.getY());
        //telemetry.addData("finalHeading", poseEstimate.getHeading());
        //telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }



    private void initTraj(SampleMecanumDrive drive, int elementPosition){
        switch(elementPosition){
            case 1:
                xValBackdrop = 22;
                break;
            case 2: xValBackdrop = 28; break;
            case 3: xValBackdrop = 34; break;

        }
        forwardPush1 = drive.trajectoryBuilder(new Pose2d())
                .forward(36)
                .build();
        back2 = drive.trajectoryBuilder(forwardPush1.end())
                .back(10)
                .build();

        splineToBackdrop3 = drive.trajectoryBuilder(back2.end())
                .splineToLinearHeading(new Pose2d(xValBackdrop, 30, Math.toRadians(90)), Math.toRadians(0))
                .build();
        strafeParkRight4 = drive.trajectoryBuilder(splineToBackdrop3.end())
                .strafeRight(50-xValBackdrop)
                .build();
        strafeParkLeft4 = drive.trajectoryBuilder(splineToBackdrop3.end())
                .strafeLeft(xValBackdrop-2)
                .build();
        forwardParkRight5 = drive.trajectoryBuilder(strafeParkRight4.end())
                .forward(20)
                .build();
        forwardParkLeft5 = drive.trajectoryBuilder(strafeParkLeft4.end())
                .forward(20)
                .build();
    }
    public void initialize(){
        // Create the TensorFlow processor by using a builder.
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------
/*
        tfod = new TfodProcessor.Builder()
               .build();
*/

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
                .setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();


        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }

        while(!isStarted() && !isStopRequested()){
            tfod.setZoom(1.0);

            //sets element position depending on the position of the detected element
            //if object isn't detected, we are assuming it is element = 3 (default right)
            double x = 0;

            try {
                if (!tfod.getRecognitions().isEmpty() ) {
                    Recognition recognition = tfod.getRecognitions().get(0);
                    //tfod.getFreshRecognitions()
                    x = (recognition.getLeft() + recognition.getRight()) / 2;

                    /*
                    //telemetry.addData("object detected:", x);
                    if (x <= 250) {
                        elementPos = 1;
                    } else if (x > 250) { //x coord for right){
                        elementPos = 2;
                    } else {
                        elementPos = 3;
                    }
                    */

                    if (x > 250 && x < 475){
                        elementPos = 2;
                    } else if (x > 475){
                        elementPos = 3;
                    } else if (x < 250){
                        elementPos = 1;
                    } else {
                        elementPos = 1;
                    }

                    telemetry.addData("finalX =>", recognition.getLeft());
                    telemetry.addData("finalY =>", recognition.getRight());
                    telemetry.addData("X Calculated =>", x);
                    telemetry.addData("Element detected =>", elementPos);
                } else {
                    elementPos = 1;
                    telemetry.addData("Nothing found... =>","");
                    telemetry.addData("Element detected =>", elementPos);
                    telemetry.update();
                    continue;
                }
                telemetry.update();

            } catch (Exception e) {

            }

        }
    }


    boolean targetFound;
    private AprilTagDetection desiredTag = null;
    int DESIRED_TAG_ID = 1;
    public void detectAprilTag(){
        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            telemetry.addData("something", "detected W");
            telemetry.update();
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
            telemetry.addData("id","matches");
            telemetry.update();

        } else {
            telemetry.addData("\n>","find valid target\n");
            telemetry.update();
        }
    }
}
