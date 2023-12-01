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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name="Arihant_RedBackdropDrive", group = "drive")
@Disabled
public class RedBackdropDrive_Arihant extends LinearOpMode {

    //april tag vars
    private DistanceSensor distanceSensor;
    private CRServo pixelMover;
    private Servo gate;

    private DcMotor linearSlideLeft   = null;
    private DcMotor linearSlideRight  = null;
    static final int targetLeft = 821;
    static final int targetRight = 840;
    private double upSpeed = .4;
    VisionPortal myVisionPortal;

    TrajectorySequence tapeTrajSequence;
    TrajectorySequence splineToBackdrop2;
    Trajectory strafeToPark4;
    TrajectoryBuilder park;
    int tagId = 0;
    int tagDistance = 0;
    /*
   elementPos for element position
       1 -> left
       2 -> mid
       3 -> right
    */
    int elementPos = 1;
    int xValBackdrop = 1;

    //vars for object detection
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "Red_Cube.tflite";
    private static final String[] LABELS = {
            "RedProp"
    };
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;
    Trajectory perfectBack3;
    Trajectory perfecterBack3;
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
        gate = hardwareMap.get(Servo.class, "gate");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "dist");

        pixelMover.setDirection(CRServo.Direction.REVERSE);



        //method for arm and stuff

        //will run while the code hasn't been run
        initialize();

        //USE ONLY FOR TESTING, overrides element position to hard set value
        elementPos = 1;

        telemetry.addData("element position", elementPos);
        telemetry.update();

        initTraj(drive, elementPos);

        if (isStopRequested()) return;
        linearSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        runArm(upSpeed, 138, 136);

        //go to tape using trajectories from switch case (check initialize)
        drive.followTrajectorySequence(tapeTrajSequence);

        //turn off the tfod processor
        myVisionPortal.setProcessorEnabled(tfod, false);

        //drop off pixel
        //servoPixel.setPosition(1);

        //spline to the backdrop using trajectories from switch case (check initialize)
        drive.followTrajectorySequence(splineToBackdrop2);
/*
        double desiredDistance = 3.75;

        double perfectDistance = distanceSensor.getDistance(DistanceUnit.INCH) - desiredDistance;
        if(perfectDistance >= 0){
            perfectBack3 = drive.trajectoryBuilder(splineToBackdrop2.end())
                    .back(perfectDistance)
                    .build();
        }else{
            perfectBack3 = drive.trajectoryBuilder(splineToBackdrop2.end())
                    .forward(Math.abs(perfectDistance))
                    .build();
        }


        double newYPosition = 32 + desiredDistance;

        drive.followTrajectory(perfectBack3);
        sleep(500);
        //if too close despite perfecting, do it again
        if(distanceSensor.getDistance((DistanceUnit.INCH)) < 3.6){
            perfectDistance = distanceSensor.getDistance(DistanceUnit.INCH) - desiredDistance;
            perfecterBack3 = drive.trajectoryBuilder(perfectBack3.end())
                    // doing -.05 in case it goes too far
                    .forward(Math.abs(perfectDistance)-.05)
                    .build();
            //shouldn't matter in terms of pose and stuff
            drive.followTrajectory(perfecterBack3);
        }
*/


        //set pixel servo to original position
        //servoPixel.setPosition(0);

        //turn on april tag processor and detect april tags
        myVisionPortal.setProcessorEnabled(aprilTag, true);

        centerToAprilTag();
        sleep(1000);

        telemetry.addData("distance:", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();


        runArm(upSpeed, targetLeft-138, targetRight-136);

        //gate.setPosition(1);
        telemetry.addData("gate get position 1::", gate.getPosition());
        telemetry.update();
        //sleep(1000);
        //pixelMover.setPower(1);
        telemetry.addData("Pixel mover get power 1::", pixelMover.getPower());
        telemetry.update();

        sleep(2000);
        pixelMover.setPower(1);
        pixelMover.setPower(1);

        telemetry.addData("Pixel mover get power 2::", pixelMover.getPower());
        telemetry.update();
        sleep(2000);
        //gate.setPosition(1);
        gate.setPosition(.135);

        sleep(3000);
        telemetry.addData("gate get position 2:", gate.getPosition());
        telemetry.update();
        //gate.setPosition(.135);
        //sleep(2000);
        //pixelMover.setPower(1);

        telemetry.addData("Pixel mover get power 3::", pixelMover.getPower());
        telemetry.update();
        //pixelMover.setPower(1);
        //sleep(1000);
/*
        strafeToPark4 = drive.trajectoryBuilder(perfectBack3.end())
                .lineToConstantHeading(new Vector2d(3, newYPosition))
                .build();
        drive.followTrajectory(strafeToPark4);

 */
        //runArm(upSpeed, -(targetLeft-138) , -(targetRight-136));

        //turn off april tag processor
        myVisionPortal.setProcessorEnabled(aprilTag, false);

        //park in proper space based on object detection
        //drive.followTrajectorySequence(park);



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
                tapeTrajSequence = drive.trajectorySequenceBuilder(new Pose2d())
                        .back(27)
                        .turn(Math.toRadians(90))
                        .back(4)
                        .build();
                //drop pixel

                splineToBackdrop2 = drive.trajectorySequenceBuilder(tapeTrajSequence.end())//.plus(new Pose2d(0, 0, Math.toRadians(90))))
                        .forward(2)
                        .lineToLinearHeading(new Pose2d(-32,32,Math.toRadians(180)))
                        //.splineToLinearHeading(new Pose2d(-32, 32), Math.toRadians(0))
                        .build();
                //set servo back to 0
                break;
            // -6 x val change per april tag

            case 2:
                tapeTrajSequence = drive.trajectorySequenceBuilder(new Pose2d())
                        .back(28)
                        .back(2)
                        .build();
                //drop pixel
                splineToBackdrop2 = drive.trajectorySequenceBuilder(tapeTrajSequence.end())
                        .forward(5)
                        .turn(Math.toRadians(-90))
                        .lineToConstantHeading(new Vector2d(-26, 32))
                        .build();
                //set servo back to 0
                break;
            case 3:
                tapeTrajSequence = drive.trajectorySequenceBuilder(new Pose2d())
                        .back(27)
                        .turn(Math.toRadians(-90))
                        .back(2)
                        .build();
                //drop pixel
                splineToBackdrop2 = drive.trajectorySequenceBuilder(tapeTrajSequence.end())
                        .forward(4)
                        .strafeLeft(20)
                        .splineToConstantHeading(new Vector2d(-20, 32), Math.toRadians(180))
                        .build();
                //perfect back
                break;


                //.splineToConstantHeading(new Vector2d(-20, 34.5), Math.toRadians(180))

        }

        park = drive.trajectoryBuilder(splineToBackdrop2.end())
                .lineToConstantHeading(new Vector2d());
        //-15, 32


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
            tfod.setZoom(2);


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

        linearSlideLeft.setDirection(DcMotor.Direction.REVERSE);
        linearSlideRight.setDirection(DcMotor.Direction.FORWARD);
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
