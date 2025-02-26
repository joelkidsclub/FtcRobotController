package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name="BlueWingV3Old", group = "drive")
@Disabled
public class DAutoLeftBlueWingV3Old extends LinearOpMode {
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
    boolean pixelDropped = false;

    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    //private AprilTagProcessor aprilTag2;
    SampleMecanumDrive drive;
    private DistanceSensor distanceSensor;
    private CRServo pixelMover;
    private Servo gate;
    private Servo pixelDropper;
    private DcMotor linearSlideLeft   = null;
    private DcMotor linearSlideRight  = null;
    static final int targetLeft = 771;
    static final int targetRight = 790;
    private double upSpeed = .8;
    boolean pixelBoxUp = false;

    private ElapsedTime stateTime = new ElapsedTime();  // Time into current state

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
    //private VisionPortal visionPortal2;
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
        STATE_LEFT_POS1_STEP7,
        STATE_LEFT_POS1_STEP8,
        STATE_LEFT_POS2_STEP1,
        STATE_LEFT_POS2_STEP2,
        STATE_LEFT_POS2_STEP3,
        STATE_LEFT_POS2_STEP4,
        STATE_LEFT_POS2_STEP5,
        STATE_LEFT_POS2_STEP6,
        STATE_LEFT_POS2_STEP7,
        STATE_LEFT_POS2_STEP8,
        STATE_LEFT_POS3_STEP1,
        STATE_LEFT_POS3_STEP2,
        STATE_LEFT_POS3_STEP3,
        STATE_LEFT_POS3_STEP4,
        STATE_LEFT_POS3_STEP5,
        STATE_LEFT_POS3_STEP6,
        STATE_LEFT_POS3_STEP7,
        STATE_LEFT_POS3_STEP8,
        STATE_LEFT_POS4_STEP1,
        STATE_LEFT_POS4_STEP2,
        STATE_POS_REALIGN,
        STATE_PARK,//
        IDLE//
    }

    //Trajectory traj_INITIAL;
    Trajectory traj_INITIAL_1;
    Trajectory traj_INITIAL_2;
    Trajectory traj_INITIAL_3;

    Trajectory traj_STATE_LEFT_POS1_STEP1;
    Trajectory traj_STATE_LEFT_POS1_STEP2;
    Trajectory traj_STATE_LEFT_POS1_STEP3;
    Trajectory traj_STATE_LEFT_POS1_STEP4;
    Trajectory traj_STATE_LEFT_POS1_STEP5;
    Trajectory traj_STATE_LEFT_POS1_STEP6;
    Trajectory traj_STATE_LEFT_POS1_STEP7;
    Trajectory traj_STATE_LEFT_POS1_STEP8;
    Trajectory traj_STATE_LEFT_POS2_STEP1;
    Trajectory traj_STATE_LEFT_POS2_STEP2;
    Trajectory traj_STATE_LEFT_POS2_STEP3;
    Trajectory traj_STATE_LEFT_POS2_STEP3b;
    Trajectory traj_STATE_LEFT_POS2_STEP4;
    Trajectory traj_STATE_LEFT_POS2_STEP5;
    Trajectory traj_STATE_LEFT_POS2_STEP6;
    Trajectory traj_STATE_LEFT_POS2_STEP7;
    Trajectory traj_STATE_LEFT_POS2_STEP8;
    Trajectory traj_STATE_LEFT_POS3_STEP1;
    Trajectory traj_STATE_LEFT_POS3_STEP1b;

    Trajectory traj_STATE_LEFT_POS3_STEP2;
    Trajectory traj_STATE_LEFT_POS3_STEP3;
    Trajectory traj_STATE_LEFT_POS3_STEP4;
    Trajectory traj_STATE_LEFT_POS3_STEP5;
    Trajectory traj_STATE_LEFT_POS3_STEP6;
    Trajectory traj_STATE_LEFT_POS3_STEP7;
    Trajectory traj_STATE_LEFT_POS3_STEP8;

    State currentState = State.STATE_INITIAL;
    int ver = 1;
    public int desiredTagId = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.

    //****************************************
    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    boolean rangeAchieved = false;
    double  desiredDrive    = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.1;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.1;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    //****************************************

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        pixelDropper = hardwareMap.get(Servo .class, "pixeldrop");
        pixelMover = hardwareMap.get(CRServo .class, "boxmover");
        linearSlideLeft  = hardwareMap.get(DcMotor .class, "LLS");
        linearSlideRight = hardwareMap.get(DcMotor.class, "RLS");
        gate = hardwareMap.get(Servo.class, "gate");
        //distanceSensor = hardwareMap.get(DistanceSensor .class, "dist");

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected

        boolean armUp = false;

        elementPos = initialize();
        //elementPos = 3; //Hardcoded for testing
        elementPos = 4; //Hardcoded for testing

        if (elementPos == 1) {
            desiredTagId = 1;
        }

        if (elementPos == 2) {
            desiredTagId = 2;
        }

        if (elementPos == 3) {
            desiredTagId = 3;
        }

        if (elementPos == 4) {
            desiredTagId = 3;
        }


        telemetry.addData("Element position =>", elementPos);
        telemetry.addData("Desired tag =>", elementPos);
        telemetry.update();

        //runArm(upSpeed, 138, 136);

        runArm(upSpeed, 238, 236);

        linearSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        currentState = State.STATE_INITIAL;

        if (isStopRequested()) return;

        boolean done = false;
        pixelDropper.setPosition(0.85);

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
                        else if (elementPos == 4)
                            currentState = State.STATE_LEFT_POS4_STEP1;
                        else
                            currentState = State.STATE_LEFT_POS2_STEP1;

                        //drive.followTrajectoryAsync(traj_Drive_To_Low_Junction);
                        telemetry.addData("STEP 1: STATE_INITIAL. Next step=> ",currentState);
                        telemetry.addData("Element position => ",elementPos);
                        telemetry.update();
                        //visionPortal.setProcessorEnabled(tfod, false);
                        //visionPortal.setProcessorEnabled(aprilTag, false);

                    }
                    //closeGate();
                    break;
                case STATE_LEFT_POS1_STEP1:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS1_STEP2;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_INITIAL_1);
                    }
                    break;
                case STATE_LEFT_POS1_STEP2:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS1_STEP3;//STATE_LEFT_POS1_STEP3;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS1_STEP2);
                        sleep(1000);
                        dropPurple(1.5);
                        purpleToInitial(1);
                    }
                    break;
                case STATE_LEFT_POS1_STEP3:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS1_STEP4;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS1_STEP4);
                    }
                    stateTime.reset();
                    while (stateTime.time() < 8 && opModeIsActive())  {
                        telemetry.addData("Waiting for alliance to complete their path...", String.format("%4.1f ", stateTime.time()));
                    }
                    break;
                case STATE_LEFT_POS1_STEP4:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS1_STEP5;
                        telemetry.addData("nextState => ", currentState);
                    }
                    break;
                case STATE_LEFT_POS1_STEP5:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS1_STEP6;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS1_STEP5);
                    }
                    break;
                case STATE_LEFT_POS1_STEP6:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS1_STEP7;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS1_STEP6);

                        if(!armUp) {
                            runArm(upSpeed, targetLeft - 138, targetRight - 136);
                            armUp = true;
                        }
                        dropPixel();
                        //sleep(30000);
                    }

                    break;
                case STATE_LEFT_POS1_STEP7:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS1_STEP8;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS1_STEP7);
                    }
                    break;
                case STATE_LEFT_POS1_STEP8:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS_REALIGN;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS1_STEP8);
                    }
                    sleep(30000);
                    break;
                // Position 2
                case STATE_LEFT_POS2_STEP1:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS2_STEP2;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_INITIAL_2);
                    }
                    break;
                case STATE_LEFT_POS2_STEP2:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS2_STEP3;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS2_STEP2);
                        sleep(1000);
                        dropPurple(1.5);
                        purpleToInitial(1);
                    }
                    break;
                case STATE_LEFT_POS2_STEP3:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS2_STEP4;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS2_STEP4);
                    }
                    stateTime.reset();
                    while (stateTime.time() < 8 && opModeIsActive())  {
                        telemetry.addData("Waiting for alliance to complete their path...", String.format("%4.1f ", stateTime.time()));
                    }
                    break;
                case STATE_LEFT_POS2_STEP4:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS2_STEP5;
                        telemetry.addData("nextState => ", currentState);
                    }
                    break;
                case STATE_LEFT_POS2_STEP5:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS2_STEP6;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS2_STEP5);
                    }
                    break;
                case STATE_LEFT_POS2_STEP6:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS2_STEP7;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS2_STEP6);

                        if(!armUp) {
                            runArm(upSpeed, targetLeft - 138, targetRight - 136);
                            armUp = true;
                        }
                        dropPixel();
                        //sleep(30000);
                    }
                    break;
                case STATE_LEFT_POS2_STEP7:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS2_STEP8;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS2_STEP7);
                    }
                    break;
                case STATE_LEFT_POS2_STEP8:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS_REALIGN;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS2_STEP8);
                    }
                    sleep(30000);
                    break;
                // Position 3
                case STATE_LEFT_POS3_STEP1:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS3_STEP2;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_INITIAL_3);
                        sleep(1000);
                        dropPurple(1.5);
                        purpleToInitial(1);
                    }
                    break;
                case STATE_LEFT_POS3_STEP2:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS3_STEP3;//STATE_LEFT_POS1_STEP3;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS3_STEP2);
                    }
                    stateTime.reset();
                    while (stateTime.time() < 8 && opModeIsActive())  {
                        telemetry.addData("Waiting for alliance to complete their path...", String.format("%4.1f ", stateTime.time()));
                    }
                    break;
                case STATE_LEFT_POS3_STEP3:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS3_STEP4;
                        telemetry.addData("nextState => ", currentState);
                    }
                    break;
                case STATE_LEFT_POS3_STEP4:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS3_STEP5;
                        telemetry.addData("nextState => ", currentState);
                    }
                    break;
                case STATE_LEFT_POS3_STEP5:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS3_STEP6;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS3_STEP5);
                    }
                    break;
                case STATE_LEFT_POS3_STEP6:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS3_STEP7;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS3_STEP6);

                        if(!armUp) {
                            runArm(upSpeed, targetLeft - 138, targetRight - 136);
                            armUp = true;
                        }
                        dropPixel();
                        //sleep(30000);
                    }
                    break;
                case STATE_LEFT_POS3_STEP7:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_LEFT_POS3_STEP8;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS3_STEP7);
                    }
                    break;
                case STATE_LEFT_POS3_STEP8:
                    telemetry.addData("currentState => ", currentState);
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS_REALIGN;
                        telemetry.addData("nextState => ", currentState);
                        drive.followTrajectory(traj_STATE_LEFT_POS3_STEP8);
                    }
                    sleep(30000);
                    break;
                // Position 4
                case STATE_LEFT_POS4_STEP1:
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS_REALIGN;
                        stateTime.reset();
                        telemetry.addData("Time 1=>", String.format("%4.1f ", stateTime.time()));
                        telemetry.addData("Armup? pre => ", armUp);;
                        //dropPixel();
                        telemetry.update();
                    }
                    break;
                case STATE_LEFT_POS4_STEP2:
                    if (!drive.isBusy()) {
                        currentState = State.STATE_POS_REALIGN;
                        stateTime.reset();
                        telemetry.addData("0", String.format("%4.1f ", stateTime.time()) + currentState.toString());
                        telemetry.addData("nextState => ", currentState);
                        telemetry.addData("Armup? pre => ", armUp);
                    }
                    break;
                case STATE_POS_REALIGN:
                    if (!drive.isBusy()) {
                        currentState = State.STATE_PARK;
                        telemetry.addData("STEP 98: STATE_POS_REALIGN: currentState => ", currentState);
                        //visionPortal.setProcessorEnabled(tfod, false);
                        //visionPortal.setProcessorEnabled(aprilTag, true);
                        //visionPortal.setProcessorEnabled(tfod, false);
                        //visionPortal.setProcessorEnabled(aprilTag, false);
                        //visionPortal.close();
                        //telemetry.update();
                        detectAprilTag();
                        //telemetryAprilTag();
                        telemetry.addData("STEP 98: STATE_POS_REALIGN: nextState => ", currentState);
                        telemetry.update();
                    }
                    //done = true;
                    sleep(30000);
                    break;
                case STATE_PARK:
                    step = 99;
                    //visionPortal.setProcessorEnabled(tfod, false);
                    visionPortal.setProcessorEnabled(aprilTag, false);
                    visionPortal.close();
                    if (!drive.isBusy()) {
                        telemetry.addData("STEP 99: STATE_PARK: currentState => ", currentState);
                        currentState = State.IDLE;
                        telemetry.addData("STEP 99: STATE_PARK", "...");
                        telemetry.addData("STEP 99: STATE_PARK: nextState => ", currentState);
                        telemetry.update();
                    }
                    break;
                case IDLE:
                    step = 100;
                    //Do Nothing
                    if (!drive.isBusy()) {
                        done = true;
                        telemetry.addData("STEP 100: STATE_IDLE. Version =>", ver);
                        telemetry.update();
                    }
                    break;
            } //End switch

            drive.update();
            telemetry.addData("Current Pose X =>", drive.getPoseEstimate().getX());
            telemetry.addData("Current Pose Y =>", drive.getPoseEstimate().getY());
            telemetry.addData("Current Pose Heading =>", drive.getPoseEstimate().getHeading());

        } //End while
        telemetry.update();

    } //End runopmode

    public void dropPurpleOld(double tTimeSec){
        stateTime.reset();
        while (stateTime.time() < tTimeSec && opModeIsActive()) {
            telemetry.addData("Time drop purple=>", String.format("%4.1f ", stateTime.time()));
            pixelDropper.setPosition(.4);
        }
    }

    public void dropPurple(double tTimeSec){
        stateTime.reset();
        while (stateTime.time() < tTimeSec && opModeIsActive()) {
            telemetry.addData("Time drop purple=>", String.format("%4.1f ", stateTime.time()));
            pixelDropper.setPosition(0.4);
        }
    }

    public void purpleToInitial(double tTimeSec){
        pixelDropper.setPosition(0.85);//0 original
        /*
        stateTime.reset();
        while (stateTime.time() < tTimeSec && opModeIsActive()) {
            telemetry.addData("Time purple to initial=>", String.format("%4.1f ", stateTime.time()));
          pixelDropper.setPosition(0);
        }
         */
    }

    public void dropPixel() {
        if (!pixelDropped) {
            movePixelBoxToDrop(2.75);
            openGateServo(3);
            movePixelBoxToIntake(1.5);
            pixelDropped = true;

        }
    }

    public void closeGate() {
        gate.setPosition(.88);
    }

    public void openGateServo(double tTimeSec){
        stateTime.reset();
        while (stateTime.time() < tTimeSec && opModeIsActive())  {
            telemetry.addData("Time 6=>", String.format("%4.1f ", stateTime.time()));
            gate.setPosition(.135);
        }
    }

    public void closeGateServo(double tTimeSec){
        stateTime.reset();
        while (stateTime.time() < tTimeSec && opModeIsActive()) {
            telemetry.addData("Time 7=>", String.format("%4.1f ", stateTime.time()));
            gate.setPosition(1);
        }
    }

    public void movePixelBoxToDrop(double tTimeSec){
        stateTime.reset();
        while (stateTime.time() < tTimeSec && opModeIsActive()) {
            telemetry.addData("Time 8=>", String.format("%4.1f ", stateTime.time()));
            pixelMover.setPower(-1);
        }
    }

    public void movePixelBoxToIntake(double tTimeSec){
        stateTime.reset();
        while (stateTime.time() < tTimeSec && opModeIsActive()) {
            telemetry.addData("Time 9=>", String.format("%4.1f ", stateTime.time()));
            pixelMover.setPower(1);
        }
    }


    private void initTfod() {

    }   // end method initTfod()

    public int initialize(){

        int tfodEP = 3;
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
                    .addProcessors(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(aprilTag)
                    .build();
        }

        //visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);

        drive.setPoseEstimate(new Pose2d(0,0,0));

        long timeOut = (long) 0.1;

        traj_INITIAL_1 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(-26.5,10, Math.toRadians(0)))
                .build();

        traj_STATE_LEFT_POS1_STEP2 = drive.trajectoryBuilder(traj_INITIAL_1.end())
                .lineToLinearHeading(new Pose2d(-26.5,-1.5, Math.toRadians(90)))
                .build();

        traj_STATE_LEFT_POS1_STEP4 = drive.trajectoryBuilder(traj_STATE_LEFT_POS1_STEP2.end())
                .lineToLinearHeading(new Pose2d(-50.5,10, Math.toRadians(90)))
                .build();

        traj_STATE_LEFT_POS1_STEP5 = drive.trajectoryBuilder(traj_STATE_LEFT_POS1_STEP4.end())
                //.back(85)
                .lineToLinearHeading(new Pose2d(-50.5,-75, Math.toRadians(90)))
                .build();

        traj_STATE_LEFT_POS1_STEP6 = drive.trajectoryBuilder(traj_STATE_LEFT_POS1_STEP5.end())
                //.strafeRight(22)
                .lineToLinearHeading(new Pose2d(-19.6,-83.50, Math.toRadians(90)))
                .build();

        traj_STATE_LEFT_POS1_STEP7 = drive.trajectoryBuilder(traj_STATE_LEFT_POS1_STEP6.end())
                .strafeLeft(31)
                .build();

        traj_STATE_LEFT_POS1_STEP8 = drive.trajectoryBuilder(traj_STATE_LEFT_POS1_STEP7.end())
                .back(8)
                .build();

// Position 2
        traj_INITIAL_2 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(-25.5,20, Math.toRadians(0)))
                .build();

        traj_STATE_LEFT_POS2_STEP2 = drive.trajectoryBuilder(traj_INITIAL_2.end())
                .lineToLinearHeading(new Pose2d(-35,14, Math.toRadians(90)))
                .build();

        traj_STATE_LEFT_POS2_STEP4 = drive.trajectoryBuilder(traj_STATE_LEFT_POS2_STEP2.end())
                .lineToLinearHeading(new Pose2d(-50.5,20, Math.toRadians(90)))
                .build();

        traj_STATE_LEFT_POS2_STEP5 = drive.trajectoryBuilder(traj_STATE_LEFT_POS2_STEP4.end())
                //.back(96)
                .lineToLinearHeading(new Pose2d(-50.5,-75, Math.toRadians(90)))
                .build();
        traj_STATE_LEFT_POS2_STEP6 = drive.trajectoryBuilder(traj_STATE_LEFT_POS2_STEP5.end())
                //.strafeRight(22)
                .lineToLinearHeading(new Pose2d(-26.25,-83.5, Math.toRadians(90)))
                .build();
        traj_STATE_LEFT_POS2_STEP7 = drive.trajectoryBuilder(traj_STATE_LEFT_POS2_STEP6.end())
                .strafeLeft(25)
                .build();
        traj_STATE_LEFT_POS2_STEP8 = drive.trajectoryBuilder(traj_STATE_LEFT_POS2_STEP7.end())
                .back(6)
                .build();
// Position 3
        traj_INITIAL_3 = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(-28,20, Math.toRadians(90)))
                .build();

        traj_STATE_LEFT_POS3_STEP2 = drive.trajectoryBuilder(traj_INITIAL_3.end())
                .lineToLinearHeading(new Pose2d(-50.5,23, Math.toRadians(90)))
                .build();

        traj_STATE_LEFT_POS3_STEP5 = drive.trajectoryBuilder(traj_STATE_LEFT_POS3_STEP2.end())
                //.back(96)
                .lineToLinearHeading(new Pose2d(-50.5,-75, Math.toRadians(90)))
                .build();
        traj_STATE_LEFT_POS3_STEP6 = drive.trajectoryBuilder(traj_STATE_LEFT_POS3_STEP5.end())
                //.strafeRight(22)
                .lineToLinearHeading(new Pose2d(-32,-83.5, Math.toRadians(90)))
                .build();
        traj_STATE_LEFT_POS3_STEP7 = drive.trajectoryBuilder(traj_STATE_LEFT_POS3_STEP6.end())
                .strafeLeft(19.5)
                .build();
        traj_STATE_LEFT_POS3_STEP8 = drive.trajectoryBuilder(traj_STATE_LEFT_POS3_STEP7.end())
                .back(12)
                .build();

        while(!isStarted() && !isStopRequested()){
            initTfod();
            tfod.setZoom(1.25);
            tfodEP = 3;
            //sets element position depending on the position of the detected element
            //if object isn't detected, we are assuming it is element = 3 (default right)
            double x = 0;
            double y = 0;

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            if(currentRecognitions.size() == 0) {
                tfodEP = 3;
            }

            for (Recognition recognition : currentRecognitions) {
                x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                if (x < 200) {
                    tfodEP = 1;
                    telemetry.addData("- Element Position Left =>", tfodEP);
                    //telemetry.update();
                } else if (x > 200 && x < 540) {
                    tfodEP = 2;
                    telemetry.addData("- Element Position Middle =>", tfodEP);
                    //telemetry.update();
                } else {
                    tfodEP = 3;
                    telemetry.addData("- Element Position Right =>", tfodEP);
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

        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FLD");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FRD");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BLD");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BRD");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);

        telemetry.addData("In Detect ATag. Looking for: ", desiredTagId);
        telemetry.update();

        int i =0;
        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur


        ElapsedTime runtime = new ElapsedTime();

        //while (runtime.seconds() < 3 && opModeIsActive()) {
        while (opModeIsActive() && runtime.time() <= 3) {
            telemetry.addLine("while loop going");
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addLine("for loop going");
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((detection.id == desiredTagId)) {
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

            telemetry.addData("Range=>", desiredTag.ftcPose.range);
            telemetry.addData("DESIRED_DISTANCE=>", DESIRED_DISTANCE);

            if(desiredTag != null && desiredTag.ftcPose.range <= DESIRED_DISTANCE ){
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                rangeAchieved = true;
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                desiredDrive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", desiredDrive, strafe, turn);
            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", desiredDrive, strafe, turn);
            }
            // Apply desired axes motions to the drivetrain.
            //moveRobot((-1)* desiredDrive, strafe, turn);
            moveRobot(desiredDrive , 0, 0);
            telemetry.update();
            sleep(10);
            drive.update();
            //break;
        }
        telemetry.addData("Exit Detect ATag", "...");
        telemetry.update();
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
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);


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
                telemetry.addLine(String.format("X, Y, Z %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("Pitch - %6.1f Rol - %6.1f Yaw - %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("Range - %6.1f Detection - %6.1f Bearing - %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
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

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
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

} //End class
