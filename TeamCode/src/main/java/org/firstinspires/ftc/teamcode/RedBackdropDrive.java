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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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
public class RedBackdropDrive extends LinearOpMode {

    //april tag vars



    private CRServo pixelMover;

    private DcMotor linearSlideLeft   = null;
    private DcMotor linearSlideRight  = null;
    static final int targetLeft = 771;
    static final int targetRight = 790;
    private double upSpeed = .4;
    VisionPortal myVisionPortal;

    TrajectorySequence tapeTrajSequence;
    TrajectorySequence splineToBackdrop2;
    TrajectorySequence park;
    int tagId = 0;
    int tagDistance = 0;
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
    private AprilTagProcessor aprilTag;
    boolean elementDetected = false;
    double  driveTag = 0;        // Desired forward power/speed (-1 to +1)
    double  strafeTag = 0;        // Desired strafe power/speed (-1 to +1)
    double  turnTag = 0;
    @Override
    public void runOpMode() throws InterruptedException {


        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        linearSlideLeft  = hardwareMap.get(DcMotor.class, "LLS");
        linearSlideRight = hardwareMap.get(DcMotor.class, "RLS");
        pixelMover = hardwareMap.get(CRServo.class, "boxmover");

        linearSlideLeft.setDirection(DcMotor.Direction.REVERSE);
        linearSlideRight.setDirection(DcMotor.Direction.FORWARD);
        pixelMover.setDirection(CRServo.Direction.REVERSE);


        //method for arm and stuff

        //will run while the code hasn't been run
        initialize();

        telemetry.addData("element position", elementPos);
        telemetry.update();

        initTraj(drive, elementPos);

        if (isStopRequested()) return;


        //go to tape using trajectories from switch case (check initialize)
        drive.followTrajectorySequence(tapeTrajSequence);

        //turn off the tfod processor
        myVisionPortal.setProcessorEnabled(tfod, false);

        //drop off pixel
        //servoPixel.setPosition(1);

        //spline to the backdrop using trajectories from switch case (check initialize)
        drive.followTrajectorySequence(splineToBackdrop2);

        //set pixel servo to original position
        //servoPixel.setPosition(0);

        //turn on april tag processor and detect april tags
        myVisionPortal.setProcessorEnabled(aprilTag, true);

        centerToAprilTag();
        sleep(4000);

        runArm(upSpeed, targetLeft, targetRight);
        sleep(1000);
        pixelMover.setPower(1);
        sleep(4000);
        pixelMover.setPower(-1);
        sleep(2000);
        runArm(upSpeed, -targetLeft, -targetRight);

        //turn off april tag processor
        myVisionPortal.setProcessorEnabled(aprilTag, false);

        //park in proper space based on object detection
        drive.followTrajectorySequence(park);



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
                xValBackdrop = 18;
                tapeTrajSequence = drive.trajectorySequenceBuilder(new Pose2d())
                        .back(27)
                        .turn(Math.toRadians(-90))
                        .back(4)
                        .build();
                //drop pixel
                splineToBackdrop2 = drive.trajectorySequenceBuilder(tapeTrajSequence.end())
                        .forward(2)
                        .splineToLinearHeading(new Pose2d(-19, 29.5, Math.toRadians(0)), Math.toRadians(180))
                        .build();
                //set servo back to 0
                break;
            // -6 x val change per april tag

            case 2:
                xValBackdrop = 28;
                tapeTrajSequence = drive.trajectorySequenceBuilder(new Pose2d())
                        .back(30)
                        .build();
                //drop pixel
                splineToBackdrop2 = drive.trajectorySequenceBuilder(tapeTrajSequence.end())
                        .forward(5)
                        .splineToLinearHeading(new Pose2d(-25, 29.5, Math.toRadians(90)), Math.toRadians(180))
                        .build();
                //set servo back to 0
                break;
            case 3:
                xValBackdrop = 35;
                tapeTrajSequence = drive.trajectorySequenceBuilder(new Pose2d())
                        .back(27)
                        .turn(Math.toRadians(90))
                        .back(4)
                        .build();
                //drop pixel
                splineToBackdrop2 = drive.trajectorySequenceBuilder(tapeTrajSequence.end())
                        .forward(2)
                        .strafeRight(10)
                        .splineToConstantHeading(new Vector2d(-31, 29.5), Math.toRadians(180))
                        .build();
                //set servo back to 0
                break;

        }

        if(elementPos < 3){
            park = drive.trajectorySequenceBuilder(splineToBackdrop2.end())
                    .lineToConstantHeading(new Vector2d(-2, -29.5))
                    .back(15)
                    .build();
        }else{
            park = drive.trajectorySequenceBuilder(splineToBackdrop2.end())
                    .lineToConstantHeading(new Vector2d(-47.5, -29.5))
                    .back(15)
                    .build();
        }
    }
    public void initialize(){
        //making april tag processor
        //servoPixel.setPosition(0);
        aprilTag = new AprilTagProcessor.Builder()
                .build();
        //making tensor flow variable, adjust tensorflow in here
        tfod = new TfodProcessor.Builder()

                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();


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
            myVisionPortal.setProcessorEnabled(tfod, true);
            tfod.setZoom(2.0);


            //sets element position depending on the position of the detected element
            //if object isn't detected, we are assuming it is element = 3 (default right)
            double xPosElement = 0;
            if (!tfod.getRecognitions().isEmpty()) {
                Recognition recognition = tfod.getRecognitions().get(0);
                xPosElement = (recognition.getLeft() + recognition.getRight()) / 2;

                //sets element position based on position of element
                if (xPosElement <= 250) {
                    elementPos = 1;
                } else if (xPosElement > 250) { //x coord for right){
                    elementPos = 2;
                }else{
                    elementPos = 3;
                }
                telemetry.addData("elementPos: ", elementPos);
                telemetry.update();
            }
        }
    }//end of initialize


    boolean targetFound;
    private AprilTagDetection desiredTag = null;

    public void centerToAprilTag(){
        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if(!currentDetections.isEmpty()){
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                telemetry.addData("something", "detected W");
                telemetry.update();
                double tagPoseX = detection.ftcPose.x;
                telemetry.addData("tag position", tagPoseX);
                telemetry.update();
                if (detection.metadata != null && tagPoseX > 220 && tagPoseX < 280) {
                    //  Check to see if we want to track towards this tag.
                    if ((detection.id <= 6) && (detection.id >= 4)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        tagId = detection.id;
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

    public void runArm(double speed, int leftTicks, int rightTicks){
        linearSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            linearSlideLeft.setTargetPosition(leftTicks);
            linearSlideRight.setTargetPosition(rightTicks);

            // Turn On RUN_TO_POSITION
            linearSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlideLeft.setPower(speed);
            linearSlideRight.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (linearSlideLeft.isBusy() && linearSlideRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", targetLeft,  targetRight);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        linearSlideLeft.getCurrentPosition(), linearSlideRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            linearSlideLeft.setPower(0);
            linearSlideRight.setPower(0);

            // Turn off RUN_TO_POSITION
            linearSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
