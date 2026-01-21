package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "BasicAuto",group ="FTC")
public class BasicAuto extends LinearOpMode {
    // Here's where we declare the Limelight & IMU
    private Limelight3A limelight = null;

    // Here's where we declare all of the wheels' motors

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;

    // Here's where we declare the operational motors:

    private DcMotor wilmerClimberLeft = null;
    private DcMotor jhoandryClimberRight = null;
    private DcMotor shooter = null;
    private DcMotor head = null;


    // Here's where we declare the operational Servos

    private CRServo intake = null;
    private CRServo secondIntake = null;
    private Servo auxiliaryShooter = null;


    // Here we declare the motors integer positions

    final int HOME_POSITION = 10;
    final int PARK_POSITION = 6000;

    // Here we declare the double positions for the servos:

    final double ARTIFACT_SHOOT = 0.5;
    final double ARTIFACT_COLLECT = 0.0;


    int wilmerPosition = HOME_POSITION;
    int jhoandryPosition = HOME_POSITION;

    @Override
    public void runOpMode() {

        /* Here's where we tell the class where to find these components
        within the Hardware Map of the Driver's Hub.
         */

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        jhoandryClimberRight = hardwareMap.get(DcMotor.class, "rightClimber");
        wilmerClimberLeft = hardwareMap.get(DcMotor.class, "leftClimber");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        head = hardwareMap.get(DcMotor.class, "head");

        // We'd be doing the same thing here, but for the servos:

        intake = hardwareMap.get(CRServo.class, "intake");
        secondIntake = hardwareMap.get(CRServo.class, "secondIntake");
        auxiliaryShooter = hardwareMap.get(Servo.class, "auxiliaryShooter");

        // Here's where we configure the Limelight and IMU hardwareMap setups:

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // april tag #20 & #24 pipeline



        /* Here's where we set the directions of the wheels
        to makes sure that all of them are going forward.
         */

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Here we set directions for the parking climbers:

        wilmerClimberLeft.setDirection(DcMotor.Direction.FORWARD);
        jhoandryClimberRight.setDirection(DcMotor.Direction.REVERSE);

        // Here we set the directions for the intakes:

        intake.setDirection(CRServo.Direction.FORWARD);
        secondIntake.setDirection(CRServo.Direction.REVERSE);

        // Here's where we set the direction of the shooter's motor & auxiliary servo:

        head.setDirection(DcMotor.Direction.REVERSE);
        auxiliaryShooter.setDirection(Servo.Direction.FORWARD);

        // Here's where we configure the Zero Power Behavior for the Climbers:

        wilmerClimberLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jhoandryClimberRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Here we program the climbers' encoders:

        wilmerClimberLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wilmerClimberLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        jhoandryClimberRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jhoandryClimberRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        limelight.start();

        while (opModeIsActive()) {
            leftFrontDrive.setPower(-1);
            rightFrontDrive.setPower(-1);
            leftRearDrive.setPower(-1);
            rightRearDrive.setPower(-1);
            sleep(500);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightRearDrive.setPower(0);
            shooter.setPower(0.75);
            intake.setPower(0.75);
            secondIntake.setPower(0.75);
            sleep(500);
            shooter.setPower(0);
            intake.setPower(0);
            secondIntake.setPower(0);
            /*
            leftFrontDrive.setPower(-1);
            rightFrontDrive.setPower(1);
            leftRearDrive.setPower(0);
            rightRearDrive.setPower(0);
            sleep(200);
            leftFrontDrive.setPower(1);
            rightFrontDrive.setPower(1);
            leftRearDrive.setPower(1);
            rightRearDrive.setPower(1);
            leftFrontDrive.setPower(1);
            intake.setPower(1);
            secondIntake.setPower(1);
            sleep(500);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightRearDrive.setPower(0);
            sleep(200);
            leftFrontDrive.setPower(-1);
            rightFrontDrive.setPower(-1);
            leftRearDrive.setPower(-1);
            rightRearDrive.setPower(-1);
            sleep(500);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightRearDrive.setPower(0);
            shooter.setPower(1);
            intake.setPower(1);
            secondIntake.setPower(1);
             */
        }

    }
}
