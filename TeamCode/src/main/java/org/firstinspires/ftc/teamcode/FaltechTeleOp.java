package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Admin on 12/3/2017.
 *
 */

@TeleOp(name = "Faltech TeleOpB", group = "7079")
public class FaltechTeleOp extends OpMode{
    FaltechRobot robot = new FaltechRobot();

    double deadzone_D = 0.1;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap,telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    public double WeightAvg(double x, double y, double z) {
        double speed_D = 0;


        if ((Math.abs(x) + Math.abs(y) + Math.abs(z))  != 0.0) {
            speed_D = ((x * Math.abs(x)) + (y * Math.abs(y)) + (z * Math.abs(z)))
                    / (Math.abs(x) + Math.abs(y) + Math.abs(z));
        }
        return (speed_D);
    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        double FwdBack_D = -gamepad1.right_stick_y;
        double Turn_D = gamepad1.left_stick_x;
        double StrafeR_D = gamepad1.right_trigger;
        double StrafeL_D = gamepad1.left_trigger;
        double FlipUp_D = gamepad2.right_trigger;
        double FlipDown_D = gamepad2.left_trigger;

        boolean CollectorIn_B = gamepad2.dpad_up;
        boolean CollectorEject_B = gamepad2.dpad_down;



        if (Math.abs(FwdBack_D) > deadzone_D){
            robot.driveTrain.go(FwdBack_D);
        }
        else if (StrafeR_D > deadzone_D){
            robot.driveTrain.strafeRight(StrafeR_D);
        }
        else if (StrafeL_D > deadzone_D ){
            robot.driveTrain.strafeLeft(StrafeL_D);
        }
        else if (Math.abs(Turn_D) > deadzone_D){
            robot.driveTrain.turnTeleOp(Turn_D);
        }
        else{
            robot.driveTrain.stop();
        }

        //Flipper
        if (FlipUp_D > deadzone_D){
            robot.flipper.flipperUp(FlipUp_D);
        }
        else if (FlipDown_D > deadzone_D){
            robot.flipper.flipperDown(FlipDown_D);
        }
        else {
            robot.flipper.stop();
        }

        //Collector
        if (CollectorIn_B){
            robot.glyphColllection.collectionIntake(1);
        }

        else if (CollectorEject_B){
            robot.glyphColllection.collectionExpel(1);
        }
        else {
            robot.glyphColllection.stop();
        }

        if (gamepad2.x){
            robot.relicArm.srvClench();
        }
        else if (gamepad2.b){
            robot.relicArm.srvOpen();
        }
        else {
            robot.relicArm.stop();
        }


        if (Math.abs(gamepad2.right_stick_y) > deadzone_D){
            robot.relicArm.mtrFwdBack(gamepad2.right_stick_y);
        }
        else {
            robot.relicArm.stop();
        }

        telemetry.addData("Right Stick Y ", FwdBack_D);
        telemetry.addData("Left Stick X", Turn_D);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.addData("Stafe Right", StrafeR_D);
        telemetry.addData("Stafe Left", StrafeL_D);
        telemetry.addData("SrvRelicPosition",robot.relicArm.srvRelicArm.getPosition());

        telemetry.update();


//        robot.driveTrain.mtrFR.setPower(WeightAvg(Forward_D,Strafe_D,-Turn_D));
//        robot.driveTrain.mtrFL.setPower(WeightAvg(Forward_D,-Strafe_D,Turn_D));
//        robot.driveTrain.mtrBR.setPower(WeightAvg(Forward_D,-Strafe_D,-Turn_D));
//        robot.driveTrain.mtrBL.setPower(WeightAvg(Forward_D,Strafe_D,Turn_D));


    }

}
