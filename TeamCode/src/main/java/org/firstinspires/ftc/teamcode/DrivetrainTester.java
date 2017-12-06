package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Admin on 11/24/2017.
 */
@Autonomous(name = "DrivetrainTester", group = "7079")
public class DrivetrainTester extends LinearOpMode {
    FaltechRobot robot = new FaltechRobot();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Say", "About to init");
        telemetry.update();
        robot.init(hardwareMap,telemetry);
        boolean finished = false;
        waitForStart();
        while (opModeIsActive() && finished == false ) {

            robot.driveTrain.strafeRight( 0.5);
            sleep(3000);
            robot.driveTrain.stop();
            robot.driveTrain.strafeLeft( 0.5);
            sleep(3000);
            robot.driveTrain.stop();

//            robot.driveTrain.turnRight(.3,90,5);

//            robot.driveTrain.motor_test(robot);
//
//            telemetry.addData("About to", "Move Arm Left");
//            telemetry.update();
//            robot.jewelArm.moveJewelArmLeft(1,2);
//
//            telemetry.addData("About to", "Stop Arm");
//            telemetry.update();
//            robot.jewelArm.stopJewelArm();
//            robot.jewelArm.robot_sleep(1000);
//
//            telemetry.addData("About to", "Move Arm Right");
//            telemetry.update();
//            robot.jewelArm.moveJewelArmRight(1,2);
//
//            telemetry.addData("About to", "Intake Glyph");
//            telemetry.update();
//            robot.glyphColllection.glyphIntake(1,2);
//
//            telemetry.addData("About to", "Stop Collection");
//            telemetry.update();
//            robot.glyphColllection.collectionStop();
//            sleep(250);
//
//
//            telemetry.addData("About to", "Expel Glyph");
//            telemetry.update();
//            robot.glyphColllection.glyphExpel(1,2);
//
//            telemetry.addData("About to", "Flip Up");
//            telemetry.update();
//            robot.flipper.vexFlipperUp(1,2000);
//
//
//            telemetry.addData("About to", "Stop Flipper");
//            telemetry.update();
//            robot.flipper.flipperStop();
//            robot.flipper.robot_sleep(1000);
//
//            telemetry.addData("About to", "Go Down");
//            telemetry.update();
//            robot.flipper.vexFlipperDown(1,2000);


            finished = true;
        }
        robot.robotStop();
    }
}
