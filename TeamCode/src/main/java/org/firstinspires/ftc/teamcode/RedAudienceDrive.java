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
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
//@Disabled
public class RedAudienceDrive extends LinearOpMode {

    Trajectory forwardLeft1;
    Trajectory forwardPixel2;
    Trajectory backPixel3;
    Trajectory strafeRight4;
    Trajectory forwardBoundary5;
    Trajectory splineToBackdrop6;
    Trajectory forwardMiddle1;
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
        telemetry.addData("element position", elementPos);
        telemetry.update();

        initTraj(drive, elementPos);


        if (isStopRequested()) return;

        drive.followTrajectory(forwardMiddle1);
        drive.followTrajectory(back2);
        drive.followTrajectory(splineToBackdrop3);
        sleep(5000);






        Pose2d poseEstimate = drive.getPoseEstimate();
        //telemetry.addData("finalX", poseEstimate.getX());
        //telemetry.addData("finalY", poseEstimate.getY());
        //telemetry.addData("finalHeading", poseEstimate.getHeading());
        //telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    private void initTfod() {


    }   // end method initTfod()

    private void initTraj(SampleMecanumDrive drive, int elementPosition){
        switch(elementPosition){
            case 1: xValBackdrop = 18; break;
            case 2: xValBackdrop = 28; break;
            case 3: xValBackdrop = 35; break;

        }
        //left code
        forwardLeft1 = drive.trajectoryBuilder(new Pose2d())
                .forward(25)
                .build();

        //turn drive.turn(90);
        forwardPixel2 = drive.trajectoryBuilder(forwardLeft1.end().plus(new Pose2d(25, 0, Math.toRadians(270))))
                .forward(5)
                .build();
        //PUT PIXEL
        backPixel3 = drive.trajectoryBuilder(forwardPixel2.end())
                .back(5)
                .build();
        strafeRight4 = drive.trajectoryBuilder(backPixel3.end())
                .strafeRight(20)
                .build();
        forwardBoundary5 = drive.trajectoryBuilder(strafeRight4.end())
                .forward(30)
                .build();
        splineToBackdrop6 = drive.trajectoryBuilder(forwardBoundary5)
                .splineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)), Math.toRadians(0))
                .build()
        //spline to the backdrop facing backwards (180)
        //pixel stuff

    }
    public void initialize(){
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
        }
    }
}
