package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name="AAARightAutoRedWing", group = "drive")
//@Disabled
public class RightAutoRedWing extends LinearOpMode {
    /*
    elementPos for element position
       1 -> left
       2 -> mid
       3 -> right
    */
    int elementPos = 2; //Default to middle blue
    int xValBackdrop = 2; //Default to middle blue
    //vars for object detection

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final String TFOD_MODEL_ASSET = "Red_Cube.tflite";
    private static final String[] LABELS = {
            "Blue Element"
    };
    private VisionPortal visionPortal;
    boolean elementDetected = false;
    SampleMecanumDrive drive;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal myVisionPortal;


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


    State currentState = State.STATE_INITIAL;
    int ver = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        initialize();

        elementPos = 3;

        telemetry.addData("element position", elementPos);
        telemetry.update();

        currentState = State.STATE_INITIAL;

        if (isStopRequested()) return;


        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Current state 1=> ", currentState);

            switch (currentState) {
                case STATE_INITIAL:
                    step = 0;
                    myVisionPortal.setProcessorEnabled(aprilTag, false);
                    myVisionPortal.setProcessorEnabled(tfod, true);

                    telemetry.addData("STEP 0: STATE_INITIAL - drive.isBusy() ", drive.isBusy());
                    if (!drive.isBusy()) {
                        currentState = State.STATE_STEP1_BACK25;
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
                            currentState = State.STATE_POS1_STEP1;
                        else if (elementPos == 2)
                            currentState = State.STATE_POS2_STEP1;
                        else if (elementPos == 3)
                            currentState = State.STATE_POS3_STEP1;
                        else
                            currentState = State.STATE_POS2_STEP1;

                    telemetry.addData("STEP 1: STATE_STEP1_BACK25: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_STEP1_BACK25);
                    telemetry.addData("STEP 1: traj_step1_back25 -","...");
                    //telemetry.update();
                    }
                    break;
                case STATE_POS1_STEP1:
                    step = 2;
                    telemetry.addData("STEP 2: STATE_POS1_STEP1: currentState => ", currentState);
                    currentState = State.STATE_POS1_STEP2;
                    telemetry.addData("STEP 2: STATE_POS1_STEP1: nextState => ", currentState);
                    //drive.followTrajectory(traj_STATE_POS1_STEP1);//lineToLinearHeading(new Pose2d(-25,-5, Math.toRadians(-90)))
                    drive.turn(Math.toRadians(90));
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
                    currentState = State.STATE_POS1_STEP23;
                    telemetry.addData("STEP 3: STATE_POS1_STEP22: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS1_STEP22);//forward(5)
                    break;
                case STATE_POS1_STEP23:
                    step =3;
                    telemetry.addData("STEP 3: STATE_POS1_STEP22: currentState => ", currentState);
                    currentState = State.STATE_POS1_STEP3;
                    telemetry.addData("STEP 3: STATE_POS1_STEP22: nextState => ", currentState);
                    drive.turn(Math.toRadians(180));
                    break;
                case STATE_POS1_STEP3:
                    step = 4;
                    telemetry.addData("STEP 4: STATE_POS1_STEP3: currentState => ", currentState);
                    currentState = State.STATE_POS_REALIGN;
                    telemetry.addData("STEP 4: STATE_POS1_STEP3: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS1_STEP3);//lineToLinearHeading(new Pose2d(-25,10, Math.toRadians(-180)))
                    break;
                case STATE_POS2_STEP1:
                    step = 5;
                    telemetry.addData("STEP 5: STATE_POS2_STEP1: currentState => ", currentState);
                    currentState = State.STATE_POS2_STEP2;
                    telemetry.addData("STEP 5: STATE_POS2_STEP1: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS2_STEP1);//back(5)
                    break;
                case STATE_POS2_STEP2:
                    step = 6;
                    telemetry.addData("STEP 6: STATE_POS2_STEP2: currentState => ", currentState);
                    currentState = State.STATE_POS2_STEP3;
                    telemetry.addData("STEP 6: STATE_POS2_STEP2: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS2_STEP2);//forward(5)
                    break;
                case STATE_POS2_STEP3:
                    step = 7;
                    telemetry.addData("STEP 7: STATE_POS2_STEP3: currentState => ", currentState);
                    currentState = State.STATE_POS_REALIGN;
                    telemetry.addData("STEP 7: STATE_POS2_STEP3: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS2_STEP3);//lineToLinearHeading(new Pose2d(-25,20, Math.toRadians(-90)))
                    break;
                case STATE_POS3_STEP1:
                    step = 8;
                    telemetry.addData("STEP 8: STATE_POS3_STEP1: currentState => ", currentState);
                    currentState = State.STATE_POS3_STEP2;
                    telemetry.addData("STEP 8: STATE_POS3_STEP1: nextState => ", currentState);
                    drive.turn(Math.toRadians(90));
                    //drive.followTrajectory(traj_STATE_POS3_STEP1);//lineToLinearHeading(new Pose2d(-25,10, Math.toRadians(-90)))
                    break;
                case STATE_POS3_STEP2:
                    step = 9;
                    telemetry.addData("STEP 9: STATE_POS3_STEP2: currentState => ", currentState);
                    currentState = State.STATE_POS3_STEP3;
                    telemetry.addData("STEP 9: STATE_POS3_STEP2: nextState => ", currentState);
                    drive.followTrajectory(traj_STATE_POS3_STEP1);
                    //drive.followTrajectory(traj_STATE_POS3_STEP1);//lineToLinearHeading(new Pose2d(-25,10, Math.toRadians(-90)))
                    break;
                case STATE_POS3_STEP3:
                    step = 10;
                    telemetry.addData("STEP 10: STATE_POS3_STEP3: currentState => ", currentState);
                    currentState = State.STATE_POS3_STEP4;
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
                    step = 12;
                    telemetry.addData("STEP 12: STATE_POS3_STEP5: currentState => ", currentState);
                    currentState = State.STATE_POS_REALIGN;
                    telemetry.addData("STEP 12: STATE_POS3_STEP5: nextState => ", currentState);
                    //drive.turn(Math.toRadians(180));
                    drive.followTrajectory(traj_STATE_POS3_STEP3);
                    break;
                case STATE_POS_REALIGN:
                    step = 98;
                    telemetry.addData("STEP 98: STATE_POS_REALIGN: currentState => ", currentState);
                    currentState = State.STATE_PARK;

                    myVisionPortal.setProcessorEnabled(tfod, false);
                    myVisionPortal.setProcessorEnabled(aprilTag, true);

                    telemetry.addData("STEP 98: STATE_POS_REALIGN: nextState => ", currentState);
                    break;
                case STATE_PARK:
                    step = 99;
                    telemetry.addData("STEP 99: STATE_PARK: currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
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
        traj_STATE_POS1_STEP2 = drive.trajectoryBuilder(traj_STATE_STEP1_BACK25.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .back(1)
                .build();

        traj_STATE_POS1_STEP22 = drive.trajectoryBuilder(traj_STATE_POS1_STEP2.end())
                .forward(6)
                .build();

        traj_STATE_POS1_STEP3 = drive.trajectoryBuilder(traj_STATE_POS1_STEP22.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                .back(20)
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
                .lineToLinearHeading(new Pose2d(-27,25, Math.toRadians(-90)))
                .build();

        //Position 3
        traj_STATE_POS3_STEP1 = drive.trajectoryBuilder(traj_STATE_STEP1_BACK25.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                //.lineToLinearHeading(new Pose2d(-25,10, Math.toRadians(-90)))
                .forward(21)
                .build();

        traj_STATE_POS3_STEP2 = drive.trajectoryBuilder(traj_STATE_POS3_STEP1 .end())
                //.lineToLinearHeading(new Pose2d(-25,10, Math.toRadians(-90)))
                .forward(4)
                .build();

        traj_STATE_POS3_STEP3 = drive.trajectoryBuilder(traj_STATE_POS3_STEP1 .end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                //.lineToLinearHeading(new Pose2d(-25,10, Math.toRadians(-90)))
                .back(57)//4
                .build();

        //Drop Pixel
        //traj_STATE_POS2_STEP2
        //traj_STATE_POS2_STEP3

        aprilTag = new AprilTagProcessor.Builder()
                .build();

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
        }   // end for() loop

    }   // end method telemetryTfod()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()


} //End class
