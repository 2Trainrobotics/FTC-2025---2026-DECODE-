package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleopDecode", group = "FTC")
public class TeleopDecode extends LinearOpMode {

    // Here's where we declare all of the wheels' motors

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;

    // Here's where we declare the operational motors:

//    private DcMotor wilmerClimberLeft = null;
//    private DcMotor jhoandryClimberRight = null;
    private DcMotor shooter = null;
    private DcMotor head = null;


    // Here's where we declare the operational Servos

    private CRServo intake = null;
    private CRServo secondIntake = null;

    // Here we declare the motors integer positions

    final int HOME_POSITION = 10;
    final int PARK_POSITION = 2000;

//    int wilmerPosition = HOME_POSITION;
//    int jhoandryPosition = HOME_POSITION;

    @Override
    public void runOpMode () {

        /* Here's where we tell the class where to find these components
        within the Hardware Map of the Driver's Hub.
         */

        leftFrontDrive = hardwareMap.get(DcMotor.class,"leftFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class,"leftRearDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class,"rightFrontDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class,"rightRearDrive");
//        wilmerClimberLeft = hardwareMap.get(DcMotor.class,"wilmerClimberLeft");
//        jhoandryClimberRight = hardwareMap.get(DcMotor.class,"jhoandryClimbeRight");
        shooter = hardwareMap.get(DcMotor.class,"shooter");
        head = hardwareMap.get(DcMotor.class,"head");

        // We'd be doing the same thing here, but for the servos:

        intake = hardwareMap.get(CRServo.class,"intake");
        secondIntake = hardwareMap.get(CRServo.class,"secondIntake");


        /* Here's where we set the directions of the wheels
        to makes sure that all of them are going forward.
         */

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Here we set directions for the parking climbers:

//        wilmerClimberLeft.setDirection(DcMotor.Direction.FORWARD);
//        jhoandryClimberRight.setDirection(DcMotor.Direction.FORWARD);

        // Here we set the directions for the intakes:

        intake.setDirection(CRServo.Direction.FORWARD);
        secondIntake.setDirection(CRServo.Direction.REVERSE);

        // Here's where we set the direction of the shooter's motor:

        head.setDirection(DcMotor.Direction.FORWARD);

        // Here's where we configure the Zero Power Behavior for the Climbers:

//        wilmerClimberLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        jhoandryClimberRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Here we program the climbers' encoders:

//        wilmerClimberLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        wilmerClimberLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        jhoandryClimberRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        jhoandryClimberRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            /* Here's where we declare the meaning of the drive commands:
            Strafe & turn into the way on how they would be playing their roles
            on the controller.
             */
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            /* Here's where we calculate the direction of the motors
            according to the direction of strafing.
             */

            double leftFrontPower = (x + y - rotation);
            double leftRearPower = (x - y + rotation);
            double rightFrontPower = (x - y - rotation);
            double rightRearPower = (x + y + rotation);

            // Here's where we assign the powers we declared to the motors

            leftFrontDrive.setPower(leftFrontPower);
            leftRearDrive.setPower(leftRearPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightRearDrive.setPower(rightRearPower);

            // Here's where we set the commands for all of the operational components:

            if(gamepad1.a) {
                intake.setPower(1);
                secondIntake.setPower(1);
            }

            else if(gamepad1.right_trigger > 0.2) {
                intake.setPower(1);
                secondIntake.setPower(1);
            }
            else if (gamepad1.left_trigger > 0.2) {
                shooter.setPower(1);
            }
            else if (gamepad1.left_bumper) {
                head.setPower(0.2);
            }
            else if (gamepad1.right_bumper) {
                head.setPower(0.2);
            }
            else {
                shooter.setPower(0);
                intake.setPower(0);
                secondIntake.setPower(0);
                head.setPower(0);
            }

//            if(gamepad1.right_bumper) {
//                wilmerPosition = PARK_POSITION;
//                jhoandryPosition = PARK_POSITION;
//            }
//            else if(gamepad1.left_bumper) {
//                wilmerPosition = HOME_POSITION;
//                jhoandryPosition = HOME_POSITION;
//            }
//
//            wilmerClimberLeft.setTargetPosition(wilmerPosition);
//            wilmerClimberLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            wilmerClimberLeft.setPower(0.4);
//
//            jhoandryClimberRight.setTargetPosition(jhoandryPosition);
//            jhoandryClimberRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            jhoandryClimberRight.setPower(0.4);



        }
    }
}
