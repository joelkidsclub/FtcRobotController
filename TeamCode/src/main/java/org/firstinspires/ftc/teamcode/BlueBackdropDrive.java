package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
import java.util.logging.XMLFormatter;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
//@Disabled
public class BlueBackdropDrive extends LinearOpMode {
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
    private static final String TFOD_MODEL_ASSET = "Blue_Cube.tflite";
    private static final String[] LABELS = {
            "Blue Element"
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

        drive.followTrajectory(forwardPush1);
        drive.followTrajectory(back2);
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
            case 1: xValBackdrop = 18; break;
            case 2: xValBackdrop = 28; break;
            case 3: xValBackdrop = 35; break;

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
                .strafeRight(48-xValBackdrop)
                .build();
        forwardParkRight5 = drive.trajectoryBuilder(strafeParkRight4.end())
                .forward(20)
                .build();
        strafeParkLeft4 = drive.trajectoryBuilder(splineToBackdrop3.end())
                .strafeLeft(xValBackdrop-2.5)
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

        tfod = new TfodProcessor.Builder()
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
