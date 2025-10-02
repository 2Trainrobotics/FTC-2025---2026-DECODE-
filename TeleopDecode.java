import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleopDecode", group = "FTC")
public class TeleopDecode extends LinearOpMode {

    // "Here's where we declare all of the wheels' motors

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;

    @Override
    public void runOpMode () {

        /* Here's where we tell the class where to find these components
        within the Hardware Map of the Driver's Hub.
         */

        leftFrontDrive = hardwareMap.get(DcMotor.class,"LeftFrontDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class,"leftRearDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class,"rightFrontDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class,"rightRearDrive");

        /* Here's where we set the directions of the wheels
        to makes sure that all of them are going forward.
         */

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

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
        }
    }
}
