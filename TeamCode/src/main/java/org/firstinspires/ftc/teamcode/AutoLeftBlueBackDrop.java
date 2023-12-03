package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
@Autonomous(name="AutoLeftBlueBackDrop", group = "drive")
//@Disabled
public class AutoLeftBlueBackDrop extends LinearOpMode {
    /*
    elementPos for element position
       1 -> left
       2 -> mid
       3 -> right
    */

    int elementPos = 1; //Default to middle blue
    int targetTagBlue = 2;
    int targetTagRed = 2;
    boolean targetFound = false;
    boolean elementDetected = false;
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    SampleMecanumDrive drive;
    private DistanceSensor distanceSensor;
    private CRServo pixelMover;
    private Servo gate;
    private Servo pixelDropper;
    private DcMotor linearSlideLeft   = null;
    private DcMotor linearSlideRight  = null;
    static final int targetLeft = 771;
    static final int targetRight = 790;
    private double upSpeed = .4;

    //boolean pixelDropped = false;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "Blue_Cube.tflite";
    private static final String[] LABELS = {
            "BlueProp"
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
        STATE_LEFT_POS1_STEP1,
        STATE_LEFT_POS1_STEP2,
        STATE_LEFT_POS1_STEP3,
        STATE_LEFT_POS1_STEP4,
        STATE_LEFT_POS1_STEP5,
        STATE_LEFT_POS1_STEP6,
        STATE_LEFT_POS2_STEP1,
        STATE_LEFT_POS2_STEP2,
        STATE_LEFT_POS2_STEP3,
        STATE_LEFT_POS2_STEP4,
        STATE_LEFT_POS2_STEP5,
        STATE_LEFT_POS2_STEP6,
        STATE_LEFT_POS3_STEP1,
        STATE_LEFT_POS3_STEP2,
        STATE_LEFT_POS3_STEP3,
        STATE_LEFT_POS3_STEP4,
        STATE_LEFT_POS3_STEP5,
        STATE_POS_REALIGN,
        STATE_PARK,//
        IDLE//
    }

    Trajectory traj_INITIAL;
    Trajectory traj_STATE_LEFT_POS1_STEP1;
    Trajectory traj_STATE_LEFT_POS1_STEP2;
    Trajectory traj_STATE_LEFT_POS1_STEP3;
    Trajectory traj_STATE_LEFT_POS1_STEP4;
    Trajectory traj_STATE_LEFT_POS1_STEP5;
    Trajectory traj_STATE_LEFT_POS1_STEP6;
    Trajectory traj_STATE_LEFT_POS2_STEP1;
    Trajectory traj_STATE_LEFT_POS2_STEP2;
    Trajectory traj_STATE_LEFT_POS2_STEP4;
    Trajectory traj_STATE_LEFT_POS2_STEP5;
    Trajectory traj_STATE_LEFT_POS3_STEP1;
    Trajectory traj_STATE_LEFT_POS3_STEP1b;

    Trajectory traj_STATE_LEFT_POS3_STEP2;
    Trajectory traj_STATE_LEFT_POS3_STEP4;
    Trajectory traj_STATE_LEFT_POS3_STEP5;

    AutoLeftBlueBackDrop.State currentState = AutoLeftBlueBackDrop.State.STATE_INITIAL;
    int ver = 1;
    public int desiredTagId = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        pixelDropper = hardwareMap.get(Servo .class, "pixeldrop");
        pixelMover = hardwareMap.get(CRServo .class, "boxmover");
        linearSlideLeft  = hardwareMap.get(DcMotor .class, "LLS");
        linearSlideRight = hardwareMap.get(DcMotor.class, "RLS");
        gate = hardwareMap.get(Servo.class, "gate");
        distanceSensor = hardwareMap.get(DistanceSensor .class, "dist");

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        boolean pixelDropped = false;
        boolean armUp = false;

        elementPos = initialize();
        //elementPos = 3; //Hardcoded for testing

        if (elementPos == 1) {
            desiredTagId = 1;
        }

        if (elementPos == 2) {
            desiredTagId = 2;
        }

        if (elementPos == 3) {
            desiredTagId = 3;
        }

        telemetry.addData("Element position =>", elementPos);
        telemetry.addData("Desired tag =>", elementPos);
        telemetry.update();

        runArm(upSpeed, 138, 136);
        linearSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                        if (elementPos == 1)
                            currentState = State.STATE_LEFT_POS1_STEP1;
                        else if (elementPos == 2)
                            currentState = State.STATE_LEFT_POS2_STEP1;
                        else if (elementPos == 3)
                            currentState = State.STATE_LEFT_POS3_STEP1;
                        else
                            currentState = State.STATE_LEFT_POS2_STEP1;

                        //drive.followTrajectoryAsync(traj_Drive_To_Low_Junction);
                        telemetry.addData("STEP 1: STATE_INITIAL. Next step=> ",currentState);
                        telemetry.addData("Element position => ",elementPos);
                        telemetry.update();
                    }
                    visionPortal.setProcessorEnabled(tfod, true);
                    visionPortal.setProcessorEnabled(aprilTag, false);
                    //break;

                case STATE_LEFT_POS1_STEP1:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS1_STEP2;
                        telemetry.addData("nextState => ", currentState);
                    }

                    drive.followTrajectory(traj_INITIAL);
                    drive.followTrajectory(traj_STATE_LEFT_POS1_STEP1);

                case STATE_LEFT_POS1_STEP2:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS1_STEP3;
                        telemetry.addData("nextState => ", currentState);
                    }

                    drive.followTrajectory(traj_STATE_LEFT_POS1_STEP2);
                    pixelDropper.setPosition(45.00);
                    sleep(1500);
                    //pixelDropper.setPosition(45.00);

                case STATE_LEFT_POS1_STEP3:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS1_STEP4;
                        telemetry.addData("nextState => ", currentState);
                    }

                    drive.followTrajectory(traj_STATE_LEFT_POS1_STEP3);
                   // break;

                case STATE_LEFT_POS1_STEP4:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS1_STEP5;
                        telemetry.addData("nextState => ", currentState);
                    }

                    drive.followTrajectory(traj_STATE_LEFT_POS1_STEP4);
                    if(!armUp) {
                        runArm(upSpeed, targetLeft - 138, targetRight - 136);
                        armUp = true;
                    }

                    //gate.setPosition(1);
                    //sleep(1000);
                    //pixelMover.setPower(1);
                    sleep(1000);
                    pixelMover.setPower(-1);
                    sleep(1000);
                    //gate.setPosition(1);
                    gate.setPosition(.135);
                    //pixelMover.setPower(-1);
                    sleep(2000);
                    //pixelMover.setPower(1);
                    pixelMover.setPower(1);
                    sleep(1000);

                    pixelDropped = true;

                case STATE_LEFT_POS1_STEP5:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS1_STEP6;
                        telemetry.addData("nextState => ", currentState);
                    }

                    drive.followTrajectory(traj_STATE_LEFT_POS1_STEP5);
                    //break;
                case STATE_LEFT_POS1_STEP6:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS_REALIGN;
                        telemetry.addData("nextState => ", currentState);
                    }

                    drive.followTrajectory(traj_STATE_LEFT_POS1_STEP6);
                    //break;
                // Position 2
                case STATE_LEFT_POS2_STEP1:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS2_STEP2;
                        telemetry.addData("nextState => ", currentState);
                    }
                    telemetry.update();

                    drive.followTrajectory(traj_INITIAL);
                    drive.followTrajectory(traj_STATE_LEFT_POS2_STEP1);
                    pixelDropper.setPosition(45.00);
                    sleep(1500);

                case STATE_LEFT_POS2_STEP2:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS2_STEP3;
                        telemetry.addData("nextState => ", currentState);
                    }
                    telemetry.update();

                    drive.followTrajectory(traj_STATE_LEFT_POS2_STEP2);
                    pixelDropper.setPosition(00);
                    sleep(1500);
                case STATE_LEFT_POS2_STEP3:
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS2_STEP4;
                        telemetry.addData("nextState => ", currentState);
                    }
                    if(!armUp) {
                        runArm(upSpeed, targetLeft - 138, targetRight - 136);
                        armUp = true;
                    }
                    //gate.setPosition(1);
                    //sleep(1000);
                    //pixelMover.setPower(1);
                    sleep(1000);
                    pixelMover.setPower(-1);
                    sleep(1000);
                    //gate.setPosition(1);
                    gate.setPosition(.135);
                    //pixelMover.setPower(-1);
                    sleep(2000);
                    //pixelMover.setPower(1);
                    pixelMover.setPower(1);
                    sleep(1000);

/*
                    gate.setPosition(1);
                    sleep(1000);
                    pixelMover.setPower(1);
                    sleep(2000);
                    pixelMover.setPower(-1);
                    sleep(1000);
                    gate.setPosition(1);
                    gate.setPosition(.135);
                    pixelMover.setPower(-1);
                    sleep(1000);
                    pixelMover.setPower(1);
                    pixelMover.setPower(1);
                    sleep(1000);
*/
                case STATE_LEFT_POS2_STEP4:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS2_STEP5;
                        telemetry.addData("nextState => ", currentState);
                    }
                    telemetry.update();

                    drive.followTrajectory(traj_STATE_LEFT_POS2_STEP4);
                    sleep(1500);

                case STATE_LEFT_POS2_STEP5:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS_REALIGN;
                        telemetry.addData("nextState => ", currentState);
                    }
                    telemetry.update();

                    drive.followTrajectory(traj_STATE_LEFT_POS2_STEP5);
                    sleep(1500);

                case STATE_LEFT_POS3_STEP1:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS3_STEP2;
                        telemetry.addData("nextState => ", currentState);
                    }
                    telemetry.update();

                    drive.followTrajectory(traj_INITIAL);
                    drive.followTrajectory(traj_STATE_LEFT_POS3_STEP1);
                    drive.turn(Math.toRadians(-90));
                    drive.followTrajectory(traj_STATE_LEFT_POS3_STEP1b);
                    pixelDropper.setPosition(45.00);
                    sleep(1500);
                case STATE_LEFT_POS3_STEP2:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS3_STEP3;
                        telemetry.addData("nextState => ", currentState);
                    }
                    telemetry.update();

                    drive.followTrajectory(traj_STATE_LEFT_POS3_STEP2);
                    sleep(1500);

                case STATE_LEFT_POS3_STEP3:
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS3_STEP4;
                        telemetry.addData("nextState => ", currentState);
                    }
                    if(!armUp) {
                        runArm(upSpeed, targetLeft - 138, targetRight - 136);
                        armUp = true;
                    }
                    //gate.setPosition(1);
                    //sleep(1000);
                    //pixelMover.setPower(1);
                    sleep(1000);
                    pixelMover.setPower(-1);
                    sleep(1000);
                    //gate.setPosition(1);
                    gate.setPosition(.135);
                    //pixelMover.setPower(-1);
                    sleep(2000);
                    //pixelMover.setPower(1);
                    pixelMover.setPower(1);
                    sleep(1000);

                    /*
                    gate.setPosition(1);
                    sleep(1000);
                    pixelMover.setPower(1);
                    sleep(2000);
                    pixelMover.setPower(-1);
                    sleep(1000);
                    gate.setPosition(1);
                    gate.setPosition(.135);
                    pixelMover.setPower(-1);
                    sleep(1000);
                    pixelMover.setPower(1);
                    pixelMover.setPower(1);
                    sleep(1000);
*/
                case STATE_LEFT_POS3_STEP4:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS3_STEP5;
                        telemetry.addData("nextState => ", currentState);
                    }
                    telemetry.update();

                    drive.followTrajectory(traj_STATE_LEFT_POS3_STEP4);
                    sleep(1500);
                case STATE_LEFT_POS3_STEP5:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS_REALIGN;
                        telemetry.addData("nextState => ", currentState);
                    }
                    telemetry.update();

                    drive.followTrajectory(traj_STATE_LEFT_POS3_STEP5);
                    sleep(1500);

                case STATE_POS_REALIGN:
                    step = 5;
                    telemetry.addData("STEP 98: STATE_POS_REALIGN: currentState => ", currentState);

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
                    //break;

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

    public int initialize(){
        pixelDropper.setPosition(0);
        int tfodEP = 1;
        //pixelMover.setPower(1);
        //sleep(2000);
        //gate.setPosition(1);

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


        traj_INITIAL = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .back(2)
                .build();
        traj_STATE_LEFT_POS1_STEP1 = drive.trajectoryBuilder(traj_INITIAL.end())
                .strafeRight(13)
                .build();
        traj_STATE_LEFT_POS1_STEP2 = drive.trajectoryBuilder(traj_STATE_LEFT_POS1_STEP1.end())
                .back(15)
                .build();
        traj_STATE_LEFT_POS1_STEP3 = drive.trajectoryBuilder(traj_STATE_LEFT_POS1_STEP2.end())
                .forward(5)
                .build();
        traj_STATE_LEFT_POS1_STEP4 = drive.trajectoryBuilder(traj_STATE_LEFT_POS1_STEP3.end())
                .lineToLinearHeading(new Pose2d(-19,-40.5, Math.toRadians(90)))
                .build();
        traj_STATE_LEFT_POS1_STEP5 = drive.trajectoryBuilder(traj_STATE_LEFT_POS1_STEP4.end())
                .strafeRight(24)
                .build();
        traj_STATE_LEFT_POS1_STEP6 = drive.trajectoryBuilder(traj_STATE_LEFT_POS1_STEP5.end())
                .back(10)
                .build();

// Position 2
        traj_STATE_LEFT_POS2_STEP1 = drive.trajectoryBuilder(traj_INITIAL.end())
                .lineToLinearHeading(new Pose2d(-44,-17, Math.toRadians(-90)))
                .build();

        traj_STATE_LEFT_POS2_STEP2 = drive.trajectoryBuilder(traj_STATE_LEFT_POS2_STEP1.end())
                .lineToLinearHeading(new Pose2d(-28.5,-40.5, Math.toRadians(90)))
                .build();

        traj_STATE_LEFT_POS2_STEP4 = drive.trajectoryBuilder(traj_STATE_LEFT_POS2_STEP2.end())
                .strafeRight(28)
                .build();

        traj_STATE_LEFT_POS2_STEP5 = drive.trajectoryBuilder(traj_STATE_LEFT_POS2_STEP4.end())
                .back(10)
                .build();
// Position 3
        traj_STATE_LEFT_POS3_STEP1 = drive.trajectoryBuilder(traj_INITIAL.end())
                .back(26)
                .build();
        traj_STATE_LEFT_POS3_STEP1b = drive.trajectoryBuilder(traj_STATE_LEFT_POS3_STEP1.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .back(3)
                .build();
        traj_STATE_LEFT_POS3_STEP2 = drive.trajectoryBuilder(traj_STATE_LEFT_POS3_STEP1b.end())
                .lineToLinearHeading(new Pose2d(-32,-38,Math.toRadians(90)))
                .build();
        traj_STATE_LEFT_POS3_STEP4 = drive.trajectoryBuilder(traj_STATE_LEFT_POS3_STEP2.end())
                .strafeRight(35)
                .build();
        traj_STATE_LEFT_POS3_STEP5 = drive.trajectoryBuilder(traj_STATE_LEFT_POS3_STEP4.end())
                .back(10)
                .build();

        //Drop Pixel
        //traj_STATE_POS2_STEP2
        //traj_STATE_POS2_STEP3

        while(!isStarted() && !isStopRequested()){
            initTfod();
            tfod.setZoom(1.25);
            tfodEP = 1;
            //sets element position depending on the position of the detected element
            //if object isn't detected, we are assuming it is element = 3 (default right)
            double x = 0;
            double y = 0;

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            if(currentRecognitions.size() == 0) {
                tfodEP = 1;
            }

            for (Recognition recognition : currentRecognitions) {
                 x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                 y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                if (x > 90 && x < 350) {
                    tfodEP = 2;
                    telemetry.addData("- Element Position =>", tfodEP);
                    //telemetry.update();
                } else if (x > 400) {
                    tfodEP = 3;
                    telemetry.addData("- Element Position =>", tfodEP);
                    //telemetry.update();
                } else {
                    telemetry.addData("- Element Position =>", tfodEP);
                }

                telemetry.addData(""," ");
                telemetry.addData("Image => ", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position =>", "%.0f / %.0f", x, y);
                telemetry.addData("- Size => ", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            }   // end for() loop
            telemetry.addData("- tfodEP =>",tfodEP);

            sleep(20);
            telemetry.update();
        } //End While
        return  tfodEP;
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

} //End class
