package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;

// cd /Users/linva/AppData/Local/Android/Sdk/platform-tools
// ./adb connect 192.168.43.1:5555

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
        telemetry.addLine("Hello Linval!");
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

        if (gamepad2.left_trigger != 0) {
            robot.claw.setPower(gamepad2.left_trigger*.5);
        } else if (gamepad2.right_trigger != 0) {
            robot.claw.setPower(-gamepad2.right_trigger*.5);
        } else robot.claw.setPower(0);


        if (-gamepad2.left_stick_y > 0) {
            robot.lift.setPower(-gamepad2.left_stick_y);
        } else if (-gamepad2.left_stick_y < 0 && robot.lift.getCurrentPosition() > 40) {
            robot.lift.setPower(-gamepad2.left_stick_y);
        } else robot.lift.setPower(0);


        if (gamepad2.right_bumper)
            robot.sticks.setPosition(.5);

        if (gamepad2.left_bumper)
            robot.sticks.setPosition(.45);






//        if (gamepad1.x)
//            robot.claw.setTargetPosition();
//        robot.claw.setMode(DcMotor.);

        /*if (gamepad1.right_bumper)
            robot.intake.setPower(1);
        else robot.intake.setPower(0);


        if (gamepad1.left_bumper)
            robot.intake.setPower(-1);
        else robot.intake.setPower(0);
*/

        telemetry.addData("", "Left Front %d   Right Front %d", robot.leftFront.getCurrentPosition(), robot.rightFront.getCurrentPosition());

        telemetry.addData("", "Left Back %d   Right Back %d", robot.leftBack.getCurrentPosition(), robot.rightBack.getCurrentPosition());

        telemetry.addData("", "Angle  %f", robot.getAngle());

        telemetry.addData("", "Angle  %f", -gamepad2.left_stick_y);

        telemetry.addData("", "lift %d", robot.lift.getCurrentPosition());

        telemetry.addData("", "claw %d", robot.claw.getCurrentPosition());

        telemetry.update();

        //code taken from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
        double facing = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        if (gamepad1.options) {
            robot.imu.resetYaw();
        }

        double rotX = x * Math.cos(-facing) - y * Math.sin(-facing);
        rotX = rotX * 1.1;
        double rotY = x * Math.sin(-facing) + y * Math.cos(-facing);

        double d = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double lf = (rotY + rotX + rx) / d;
        double lb = (rotY - rotX + rx) / d;
        double rf = (rotY - rotX - rx) / d;
        double rb = (rotY + rotX - rx) / d;

        //set rightFront negative so it goes same direction as other heels
        robot.leftFront.setVelocity(2000 * lf * shift);
        robot.leftBack.setVelocity(2000 * lb * shift);
        robot.rightFront.setVelocity(2000 * rf * shift);
        robot.rightBack.setVelocity(2000 * rb * shift);


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
        robot.lift.setPower(0);
        robot.claw.setPower(0);
        robot.sticks.setPosition(0);
    }

    // fuction used to make sure that the value inputted to the
    // motors stays between 1 and -1
    private double clamp(double x, double min, double max) {

        return Math.max(min,Math.min(max,x));

    }


}

