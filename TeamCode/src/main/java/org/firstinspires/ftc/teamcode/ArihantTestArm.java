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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
@Autonomous(name="ArihantTestArm", group = "drive")
@Disabled
public class ArihantTestArm extends LinearOpMode {

    private DcMotor linearSlideLeft   = null;
    private DcMotor linearSlideRight  = null;
    private double upSpeed = .8;
    private DistanceSensor distanceSensor;

    private CRServo pixelMover;
    private Servo pixelDropper;


    SampleMecanumDrive drive;



    @Override
    public void runOpMode() throws InterruptedException {
        linearSlideLeft  = hardwareMap.get(DcMotor .class, "LLS");
        linearSlideRight = hardwareMap.get(DcMotor.class, "RLS");
        distanceSensor = hardwareMap.get(DistanceSensor .class, "dist");
        pixelDropper = hardwareMap.get(Servo.class, "pixeldrop");
        pixelMover = hardwareMap.get(CRServo .class, "boxmover");

        pixelDropper.setPosition(0);

        linearSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pixelMover.setPower(1);
        pixelDropper.setPosition(45);

        runArm(upSpeed);



        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);


        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {

        } //End while
        telemetry.update();

    }
    public void runArm(double speed){
        linearSlideLeft.setDirection(DcMotor.Direction.REVERSE);
        linearSlideRight.setDirection(DcMotor.Direction.FORWARD);

        linearSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure that the OpMode is still active/run until distance
        while(distanceSensor.getDistance(DistanceUnit.CM) <= 10 && opModeIsActive()){
                linearSlideLeft.setPower(speed);
                linearSlideRight.setPower(speed);
                telemetry.addData("Currently at",  " at %7d :%7d", linearSlideLeft.getCurrentPosition(), linearSlideRight.getCurrentPosition());
                telemetry.update();
        }
        // Stop all motion;
        linearSlideLeft.setPower(0);
        linearSlideRight.setPower(0);

        pixelMover.setPower(-1);

        }
    }
