package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="A TeleOp Main", group="Linear OpMode")
//@Disabled
public class TeleOp_CB13353 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private CRServo droneServo;

/*
    private DistanceSensor distanceSensor;
    private TouchSensor touchSensor;
    private ColorSensor colorSensor;
*/

    private double speed = 1;
    private double turnspeed = 1;
    private int inverted = 1;


    private DcMotor linearSlideLeft = null;

    private DcMotor linearSlideRight = null;

    private DcMotor intake = null;

    private double upspeed;

    private Servo pixelMover;
    private Servo gate;
    private DcMotor LinearActuator;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        //Hello
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FLD");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BLD");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FRD");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BRD");
//      distanceSensor = hardwareMap.get(DistanceSensor.class, "dist");
//      touchSensor = hardwareMap.get(TouchSensor.class, "touch");
//      colorSensor = hardwareMap.get(ColorSensor.class, "color");
        linearSlideLeft = hardwareMap.get(DcMotor.class, "LLS");
        linearSlideRight = hardwareMap.get(DcMotor.class, "RLS");
        intake = hardwareMap.get(DcMotor.class, "INTAKE");
        pixelMover = hardwareMap.get(Servo.class, "boxmover");
        gate = hardwareMap.get(Servo.class, "gate");
        LinearActuator = hardwareMap.get(DcMotor.class, "LA");
        //droneServo = hardwareMap.get(CRServo.class,"droneLauncher");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        linearSlideLeft.setDirection(DcMotor.Direction.REVERSE);
        linearSlideRight.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        pixelMover.setDirection(Servo.Direction.REVERSE);
        LinearActuator.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        linearSlideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        linearSlideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pixelMover.setPosition(0.28);
        waitForStart();
        runtime.reset();

        upspeed = 0.4;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //slows robot down
            if (gamepad1.left_bumper) {
                speed = 0.25;
                turnspeed = 0.25;
            } else if (gamepad1.right_bumper) {
                speed = 1;
                turnspeed = 0.5;
            } else {
                speed = 0.5;
                turnspeed = 0.5;
            }

            if (gamepad1.square){
                inverted = -1;
            }

            if(gamepad1.circle){
                inverted = 1;
            }

            double LinearSlideMovement = gamepad2.left_stick_y * -upspeed;
            linearSlideLeft.setPower(LinearSlideMovement);
            linearSlideRight.setPower(LinearSlideMovement);

            double IntakeMovement = gamepad2.right_stick_y;
            intake.setPower(IntakeMovement);


            if (gamepad2.dpad_left){
                LinearActuator.setPower(1);
            }else if (gamepad2.dpad_right){
                LinearActuator.setPower(-1);
            }else{
                LinearActuator.setPower(0);
            }

            if (gamepad2.triangle){
                pixelMover.setPosition(0.95);
                telemetry.addData("Triangle is pressed", pixelMover.getPosition());
            }

            if(gamepad2.circle){
                pixelMover.setPosition(0.28);
                telemetry.addData("Circle is pressed", pixelMover.getPosition());
            }

            if(gamepad2.square) {
                pixelMover.setPosition(0.50);
                telemetry.addData("Square is pressed", pixelMover.getPosition());
            }

            if(gamepad2.dpad_up) {
                gate.setPosition(0.135);
            }

            if(gamepad2.dpad_down) {
                gate.setPosition(0.66);
            }
/*
            if (linearSlideLeft.getCurrentPosition() <= 0) {
                pixelMover.setPosition(0.28);
            }
            if (linearSlideRight.getCurrentPosition() <= -2000) {
                pixelMover.setPosition(0.28);
            }
*/




//            if (gamepad1.dpad_down){
            //release
//                droneServo.setPower(-4);
//            }

//            if (gamepad1.dpad_up){
            //wind back
//                droneServo.setPower(4);
//            }

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = gamepad1.left_stick_y * speed * inverted;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x * -speed* inverted;
            double yaw = gamepad1.right_stick_x * -turnspeed * inverted;


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);


//            DistanceSensor();
//
//           ColorSensor();

            telemetry.addData("LinearSlideMovementValue", LinearSlideMovement);
            telemetry.addData("Left slide", linearSlideLeft.getCurrentPosition());
            telemetry.addData("Right slide", linearSlideRight.getCurrentPosition());
            telemetry.addData("PixelMoverPosition", pixelMover.getPosition());
            telemetry.update();

        }
    }


/*
    public double DistanceSensor() {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        boolean touch = touchSensor.isPressed();
        telemetry.addData("Distance:", distance);

        if (gamepad1.circle || gamepad1.right_stick_button){
            if (distance <= 8.5) {
                telemetry.addLine("It very close bro");
                if (!gamepad1.isRumbling()) {
//                gamepad1.rumble(distance /5, distance/5,100);
//                gamepad1.rumble((int) distance * 100);
                  gamepad1.rumbleBlips((int) distance);
  //                  rumbler((int) distance);

                }
            }
        }

        if (touch) {
            telemetry.addLine("I'm being touched");
            if (!gamepad1.isRumbling()) {
                gamepad1.rumble(0, 1, 100);
            }
        }
        return distance;
    }

    public void ColorSensor() {
        colorSensor.enableLed(true);
        double colorRed = colorSensor.red();
        double colorGreen = colorSensor.green();
        double colorBlue = colorSensor.blue();

        telemetry.addLine("Color sensor things:");
        telemetry.addData("colorRed", colorRed);
        telemetry.addData("colorGreen", colorGreen);
        telemetry.addData("colorBlue", colorBlue);

        double colorSensorMultiplier = 0.9;
        if (colorRed * colorSensorMultiplier > colorBlue && colorRed * colorSensorMultiplier > colorGreen){
            telemetry.addLine("I see more red than the rest");
        }else if ((colorBlue * colorSensorMultiplier > colorRed && colorBlue * colorSensorMultiplier > colorGreen)) {
            telemetry.addLine("I see more blue than anything else");
        } else if ((colorGreen * colorSensorMultiplier > colorRed && colorGreen * colorSensorMultiplier > colorBlue)) {
            telemetry.addLine("I see more green than anything else");
        }else{
            telemetry.addLine("I don't see anything");
        }

    }

    public void rumbler(int rumbleAmount) {
        int rumbleTimes = 0;
        while (rumbleTimes < rumbleAmount){
            gamepad1.rumble(70);
            sleep(95);
            rumbleTimes += 1;
        }
    }


 */
}

