package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Decode", group = "FTC")
public class Decode extends LinearOpMode {

    // Limelight Camera located in front of the robot:

    private Limelight3A limelight = null;

    // Here we declare all Mecanum Drive Motors"

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    // Here we declare the subsystems' motors:

    private DcMotor jhoandryRightClimber = null;
    private DcMotor wilmerLeftClimber = null;
    private DcMotor shooter = null;
    private DcMotor secondIntake = null;

    // Here we declare the subsystems' servos:

    private Servo turret = null;
    private CRServo intake = null;
    private Servo kicker = null;

    // Here we declare the elevators' integer positions:

    final int HOME_POSITION = 10;
    final int PARK_POSITION = 6000;

    // Here we declare pre-set double positions for the turret:

    final double LONG_RANGE_RIGHT = 0.1;
    final double LONG_RANGE_LEFT = 0.3;
    final double TURRET_HOME_POSITION = 0.2;

    // Here we declare the double positions for the kicker:

    final double ARTIFACT_SHOOT = 1.0;
    final double ARTIFACT_COLLECT = 0.74;

    // Here we declare the turret servo with a gradual increment mode:

    final double ROTATIONAL_CLAW_STOP = 0.0;
    final double ROTATIONAL_CLAW_CLOCKWISE = 0.9;
    final double ROTATIONAL_CLAW_COUNTERCLOCKWISE = -ROTATIONAL_CLAW_CLOCKWISE;

    // Here we program the turret servo position with its default home position:

    double turretPosition = TURRET_HOME_POSITION;

    // Here we program the encoders by configuring them to their default home position:

    int wilmerPosition = HOME_POSITION;
    int jhoandryPosition = HOME_POSITION;

    @Override
    public void runOpMode () {

        /* Here's where we guide the class to find the different components
        within the Hardware Map configuration:
         */

        leftFront = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftRear = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightRear = hardwareMap.get(DcMotor.class, "rightRearDrive");
        jhoandryRightClimber = hardwareMap.get(DcMotor.class, "rightClimber");
        wilmerLeftClimber = hardwareMap.get(DcMotor.class, "leftClimber");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        secondIntake = hardwareMap.get(DcMotor.class, "secondIntake");

        // Here we do the same thing, but, for the servos:

        turret = hardwareMap.get(Servo.class, "turret");
        intake = hardwareMap.get(CRServo.class, "intake");
        kicker = hardwareMap.get(Servo.class, "kicker");

        // Here's where we configure the Limelight and IMU hardwareMap Setup:

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // april tag #20 & #24 pipeline


        /*Here, we set the direction of the Mecanum wheels to make
        sure they're moving forward relative to their installed
        positions:
         */

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Here we set the directions for the climbers:

        wilmerLeftClimber.setDirection(DcMotor.Direction.FORWARD);
        jhoandryRightClimber.setDirection(DcMotor.Direction.REVERSE);

        // Here we set the direction for the Second Intake

        secondIntake.setDirection(DcMotor.Direction.REVERSE);


        // Here's where we program the climbers' encoders

        wilmerLeftClimber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wilmerLeftClimber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        jhoandryRightClimber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jhoandryRightClimber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        limelight.start();

        while (opModeIsActive()) {

            /* Here's where we declare the meaning of the drive commands:
            Strafe & turn - through the way on how they would be playing their roles
            on the controller:
             */

            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            /* Here's where we calculate the direction of the motors
            based on the direction of the wheels while strafing:
             */

            double leftFrontPower = (x + y - rotation);
            double leftRearPower = (x - y + rotation);
            double rightFrontPower = (x - y - rotation);
            double rightRearPower = (x + y + rotation);

            // Here's where we assign the powers we declared to the motors:

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

            // Here we configure the commands for Gamepad #1:

            if (gamepad1.dpad_up) {

                wilmerPosition = PARK_POSITION;
                jhoandryPosition = PARK_POSITION;
            } else if (gamepad1.dpad_down) {

                wilmerPosition = HOME_POSITION;
                jhoandryPosition = HOME_POSITION;
            }
            if (gamepad1.left_bumper) {
                turret.setPosition(LONG_RANGE_LEFT);
            }
            else if (gamepad1.right_bumper) {
                turret.setPosition(LONG_RANGE_RIGHT);
            }

            if (gamepad1.left_trigger > 0.3) {
                intake.setPower(1);
            }

            else {
                intake.setPower(0);
            }

            // Here we configure the commands for Gamepad #2:

            if (gamepad2.right_trigger >0.3) {
                secondIntake.setPower(0.5);
            }
            else {
                secondIntake.setPower(0);
            }

            if (gamepad2.dpad_left) {
                kicker.setPosition(ARTIFACT_SHOOT);
            }
            else if (gamepad2.dpad_right) {
                kicker.setPosition(ARTIFACT_COLLECT);
            }

            if (gamepad2.a) {
                shooter.setPower(0.55);
            }

            else if(gamepad2.b) {
                shooter.setPower(0.68);
            }

            else if (gamepad2.y) {
                shooter.setPower(0.85);
            }

            else {
                shooter.setPower(0);
            }

            double turretIncrement = 0.02;

            if (gamepad2.left_bumper) {
                turretPosition += turretIncrement;
            }
            else if (gamepad2.right_bumper) {
                turretPosition -= turretIncrement;
            }

            turretPosition = Math.max(0, Math.min(1, turretPosition));
            turret.setPosition(turretPosition);

            wilmerLeftClimber.setTargetPosition(wilmerPosition);
            wilmerLeftClimber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wilmerLeftClimber.setPower(0.4);

            jhoandryRightClimber.setTargetPosition(jhoandryPosition);
            jhoandryRightClimber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            jhoandryRightClimber.setPower(0.4);


        }

    }
}
