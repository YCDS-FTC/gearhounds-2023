
package org.firstinspires.ftc.teamcode.Auton;

        import android.transition.Slide;

        import com.qualcomm.hardware.dfrobot.HuskyLens;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.teamcode.AutonCommands.MoveForDistance;
        import org.firstinspires.ftc.teamcode.AutonCommands.SetClaws;
        import org.firstinspires.ftc.teamcode.AutonCommands.SetWrist;
        import org.firstinspires.ftc.teamcode.AutonCommands.StrafeForDistance;
        import org.firstinspires.ftc.teamcode.AutonCommands.TurnByAngle;
        import org.firstinspires.ftc.teamcode.AutonCommands.WaitForTime;
        import org.firstinspires.ftc.teamcode.Utilities.Command;
        import org.firstinspires.ftc.teamcode.AutonCommands.SlideToPosition;
        import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;
        import org.firstinspires.ftc.teamcode.Utilities.GearHoundsHardware;

        import java.util.ArrayList;
        import java.util.List;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *A
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Red_Rightt")
public class Red_Rightt extends LinearOpMode {
    private GearHoundsHardware robot = new GearHoundsHardware();
    //Create elapsed time variable and an instance of elapsed time
    private ElapsedTime runtime = new ElapsedTime();
    private boolean done = false;
    private double propPos = 0;

    @Override
    public void runOpMode() {
        double startTime = 0;
        robot.init(hardwareMap);
        robot.imu.resetYaw();
        List<Command> steps = new ArrayList<>();



        int step = 0;
        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        // Waiting for start
        while(!isStarted()) {
            telemetry.update();


            HuskyLens.Block[] blocks = robot.huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }


        }
        startTime = runtime.seconds();
        HuskyLens.Block[] blocks = robot.huskyLens.blocks();
        propPos = 1;
        telemetry.addLine("On the right");
        for (int i = 0; i < blocks.length; i++) {
            if (blocks[i].id == 1 && blocks[i].y > 100) {
                if (blocks[i].x > 160 ) {
                    propPos = 2;
                    telemetry.addLine("On the Middle");
                } else {
                    propPos = 3;
                    telemetry.addLine("On the left");
                }
            }
        }
        telemetry.addData("Prop Position: ", "%d", propPos);
        telemetry.update();


        steps.add(new MoveForDistance(robot,18 ,10 , 5, runtime, 5, -0.5, 1));

        //steps.add(new StrafeForDistance(robot, 5, 1, 1, runtime, 5, 0.5, 1));

        //steps.add(new MoveForDistance(robot, 20, 5, 5, runtime, 5, 0.5, 1));

        if (propPos == 1) {
            steps.add(new MoveForDistance(robot, 3, 1, 1, runtime, 5, -0.5, 1));
            steps.add(new StrafeForDistance(robot, 9, 3, 1, runtime, 3, -0.25, 1 ));
            steps.add(new MoveForDistance(robot, 16, 6, 3, runtime, 5, 0.75, 0.75));
            steps.add(new WaitForTime(robot, runtime, 1));
            steps.add(new StrafeForDistance(robot,30  , 3, 1, runtime, 3, -0.25, 1 ));
            robot.claw.setPosition(9);
        } else if (propPos == 2) {
            steps.add(new StrafeForDistance(robot,5 ,2 , 1, runtime, 3, -0.75, 1 ));
            steps.add(new MoveForDistance(robot, 9, 1, 1, runtime, 5, -0.25, 1));
            steps.add(new MoveForDistance(robot, 5, 4, 0.5, runtime, 5, 0.75, 0.75));
            steps.add(new StrafeForDistance(robot,25 , 3, 1, runtime, 3, -0.75, 1 ));
            steps.add(new WaitForTime(robot, runtime, 1));
            robot.claw.setPosition(9);
        } else if (propPos == 3) {
            steps.add(new TurnByAngle(robot, runtime, 45, 0.5, 5));
            steps.add(new MoveForDistance(robot, 5, 1, 1, runtime, 5, -0.5, 1));
            steps.add(new WaitForTime(robot, runtime, 1));
            robot.claw.setPosition(9);
            steps.add(new MoveForDistance(robot, 5, 1, 1, runtime, 5, -0.5, 1));
            steps.add(new MoveForDistance(robot, 5, 1, 1, runtime, 5, 0.5, 1));
            steps.add(new MoveForDistance(robot, 28,25 , 3, runtime, 5, 0.5, 1));
            steps.add(new StrafeForDistance(robot,10 , 3, 1, runtime, 3, -0.75, 1 ));

        }

//        steps.add(new MoveForDistance(robot, 300, 100, 100, runtime, 5, -0.5, 1));
//        steps.add(new SlideToPosition(robot, runtime, 500, -0.35, 5));

        // This is where we build the autonomous routine
        Command currentStep = steps.get(step);
        while(opModeIsActive() && !done) {
            // Execute current command
            if(currentStep.getState() == Command.STARTING) {
                telemetry.addData("Step:", "Starting");
                currentStep.init();
            } else if (currentStep.getState() == Command.RUNNING) {
                telemetry.addData("Step:", "Running");
                currentStep.run();
            } else if (currentStep.getState() == Command.ENDING) {
                telemetry.addData("Step:", "Ending");
                currentStep.end();
            } else if (currentStep.getState() == Command.DONE) {
                telemetry.addData("Step:", "Done");
                step++;
                if (step >= steps.size()){
                    done = true;
                } else {
                    currentStep = steps.get(step);
                }
            }
            telemetry.update();
        }
    }
}