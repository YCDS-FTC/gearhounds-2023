package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;



@TeleOp(name="Mechanum", group="TeleOp")

/* This program is the robot's main TeleOp program which gets run
 * constistently every TeleOperated period. This allows for driver control
 * of the drivetrain, and operator control of all other subsytems on the robot.*/

public class Mechanum extends OpMode
{
    // Declare the hardwareMap for the robot
    private GearHoundsHardware robot = new GearHoundsHardware();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

	private double shift = 1.0;

    @Override
    public void init() {


        // Intializes the hardwareMap found in "GearHoundsHardware" class
        robot.init(hardwareMap);
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addLine("Hello Erik!");
        telemetry.update();
    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            shift = 1.0;
        }

        if (gamepad1.left_bumper) {
            shift = 0.3;
        }

        if (gamepad1.b) {
            shift = 0.25;
        }

        if (gamepad1.a) {
            shift = 0.5;
        }

        if (gamepad1.x) {
            shift = 0.75;
        }

        if (gamepad1.y) {
            shift = 1;
        }

        telemetry.addData("", "Left Front %d   Right Front %d", robot.leftFront.getCurrentPosition(), robot.rightFront.getCurrentPosition());

        telemetry.addData("", "Left Back %d   Right Back %d", robot.leftBack.getCurrentPosition(), robot.rightBack.getCurrentPosition());

        telemetry.addData("", "Angle  %f", robot.getAngle());

        telemetry.update();

        // Driver centric mechanum
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI / 4;
        robotAngle = robotAngle - Math.toRadians(angle);
        double rightX = gamepad1.right_stick_x;
        final double lf = -r * Math.cos(robotAngle) + rightX;
        final double lb = r * Math.sin(robotAngle) + rightX;
        final double rf = -r * Math.cos(robotAngle) - rightX;
        final double rb = r * Math.sin(robotAngle) - rightX;

        robot.leftFront.setPower(lf * shift);
        robot.leftBack.setPower(lb * shift);
        robot.rightFront.setPower(rf * shift);
        robot.rightBack.setPower(rb * shift);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // set all the motors to stop
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }

    // fuction used to make sure that the value inputted to the
    // motors stays between 1 and -1
    private double clamp(double x, double min, double max) {

        return Math.max(min,Math.min(max,x));

    }


}

