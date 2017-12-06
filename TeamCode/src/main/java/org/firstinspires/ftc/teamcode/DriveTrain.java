package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Admin on 11/24/2017.
 */

public class DriveTrain extends RobotPart{

    public DcMotor mtrFR = null;
    public DcMotor mtrFL = null;
    public DcMotor mtrBR = null;
    public DcMotor mtrBL = null;
// Sensor
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    
    private int offset = 10;

    public void init(HardwareMap ahwMap, Telemetry myTelemetry){

        super.init(ahwMap,myTelemetry);

        mtrFL = ahwMap.dcMotor.get("mtrFL");
        mtrFR = ahwMap.dcMotor.get("mtrFR");
        mtrBL = ahwMap.dcMotor.get("mtrBL");
        mtrBR = ahwMap.dcMotor.get("mtrBR");

        mtrFL.setDirection(DcMotor.Direction.FORWARD);
        mtrFR.setDirection(DcMotor.Direction.REVERSE);
        mtrBL.setDirection(DcMotor.Direction.FORWARD);
        mtrBR.setDirection(DcMotor.Direction.REVERSE);

        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }
    private void leftSideForward(double speed){
        mtrBL.setPower(Math.abs(speed));
        mtrFL.setPower(Math.abs(speed));
    }

    private void rightSideForward(double speed){
        mtrBR.setPower(Math.abs(speed));
        mtrFR.setPower(Math.abs(speed));
    }

    private void leftSideBack(double speed){
        mtrBL.setPower(-speed);
        mtrFL.setPower(-speed);
    }

    private void rightSideBack(double speed){
        mtrBR.setPower(-speed);
        mtrFR.setPower(-speed);
    }

    public void stop(){
        mtrBL.setPower(0);
        mtrBR.setPower(0);
        mtrFL.setPower(0);
        mtrFR.setPower(0);
    }

    private void goForward(double speed){
        leftSideForward(speed);
        rightSideForward(speed);
    }

    private void goBackward(double speed){
        leftSideBack(speed);
        rightSideBack(speed);
    }

    private void pivotRight(double speed){
        leftSideForward(speed);
        rightSideBack(speed);
    }

    private void pivotLeft(double speed){
        leftSideBack(speed);
        rightSideForward(speed);
    }

    public void strafeRight(double speed){
        mtrFR.setPower(speed);
        mtrBR.setPower(-speed);
        mtrFL.setPower(-speed);
        mtrBL.setPower(speed);
    }

    public void strafeLeft(double speed){
        mtrFR.setPower(-speed);
        mtrBR.setPower(speed);
        mtrFL.setPower(speed);
        mtrBL.setPower(-speed);
    }

    public void motor_test(){
        privateTelemetry.addData("Testing", "mtrBL");
        privateTelemetry.update();
        mtrBL.setPower(1);
        mySleep(1000);
        mtrBL.setPower(0);
        mySleep(250);

        privateTelemetry.addData("Testing", "mtrBR");
        privateTelemetry.update();
        mtrBR.setPower(1);
        mySleep(1000);
        mtrBR.setPower(0);
        mySleep(250);

        privateTelemetry.addData("Testing", "mtrFL");
        privateTelemetry.update();
        mtrFL.setPower(1);
        mySleep(1000);
        mtrFL.setPower(0);
        mySleep(250);

        privateTelemetry.addData("Testing", "mtrFR");
        privateTelemetry.update();
        mtrFR.setPower(1);
        mySleep(1000);
        mtrFR.setPower(0);
        mySleep(250);
    }

    private void setModeRobot(){
        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set them to run to position
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private ElapsedTime runtime = new ElapsedTime();
    double motor_count = 1440;
    double gear_reduction = 0.5;
    double wheel_circumference = 12.56;
    double counts_per_inch = (motor_count * gear_reduction / wheel_circumference);

    private void targetPositions(double inches){
        double BLtarget;
        double BRtarget;
        double FLtarget;
        double FRtarget;

        BLtarget = mtrBL.getCurrentPosition() + (inches * counts_per_inch);
        BRtarget = mtrBR.getCurrentPosition() + (inches * counts_per_inch);
        FLtarget = mtrFL.getCurrentPosition() + (inches * counts_per_inch);
        FRtarget = mtrFR.getCurrentPosition() + (inches * counts_per_inch);

        mtrBL.setTargetPosition((int) BLtarget);
        mtrBR.setTargetPosition((int) BRtarget);
        mtrFL.setTargetPosition((int) FLtarget);
        mtrFR.setTargetPosition((int) FRtarget);

        runtime.reset();
    }

    public void go(double power_D){
        mtrFR.setPower(power_D);
        mtrFL.setPower(power_D);
        mtrBR.setPower(power_D);
        mtrBL.setPower(power_D);
    }
    public void strafeTeleOp(double power_D){
        mtrFR.setPower(power_D);
        mtrFL.setPower(-power_D);
        mtrBR.setPower(-power_D);
        mtrBL.setPower(power_D);

    }

    public void turnTeleOp(double power_D){
        mtrFR.setPower(-power_D);
        mtrFL.setPower(power_D);
        mtrBR.setPower(-power_D);
        mtrBL.setPower(power_D);

    }
    

    // private void timeOutExit(double timeout){

    //     while ((runtime.seconds() < (timeout*1000))
    //             && (mtrBL.isBusy() &&mtrBR.isBusy()
    //             && mtrFL.isBusy() && mtrFR.isBusy())) {

    //         // Display it for the driver.
    //         privateTelemetry.addData("Path1",  "Running to target position");
    //         privateTelemetry.addData("Path2",  "Running at:",
    //                 mtrBL.getCurrentPosition(),
    //                 mtrBR.getCurrentPosition(),
    //                 mtrFL.getCurrentPosition(),
    //                 mtrFR.getCurrentPosition());
    //         privateTelemetry.update();
    //     }

    // }

    public void goForward(double inches, double speed, double timeout){
        runtime.reset();
        setModeRobot();
        targetPositions(inches);
        leftSideForward(speed);
        rightSideForward(speed);
        stop();
    }
    
    public void goBackward(double inches, double speed, double timeout){
        runtime.reset();
        setModeRobot();
        targetPositions(-inches);
        leftSideForward(speed);
        rightSideForward(speed);
        stop();
    }
    
    public void goStrafeRight(double speed, double timeout){
        runtime.reset();
        strafeRight(speed);
        mySleep(timeout);
        stop();
    }
    
    public void goStrafeLeft(double speed, double timeout){
        runtime.reset();
        strafeLeft(speed);
        mySleep(timeout);
        stop();
    }
    
    public void turnRight(double speed, double degrees, double timeout ){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        double tempHeading = angles.firstAngle;
        
        runtime.reset();
        pivotRight(speed);
        while (Math.abs(tempHeading - heading) <= (degrees - offset) && runtime.seconds() < timeout) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;
            double roll = angles.thirdAngle;
            double pitch = angles.secondAngle;
            privateTelemetry.addData("heading", heading);
            privateTelemetry.addData("roll", roll);
            privateTelemetry.addData("pitch", pitch);
            privateTelemetry.update();
        }
        
        if (runtime.seconds() >= timeout)
        {
            privateTelemetry.addData("timeOut elaspsed", runtime.seconds());
            privateTelemetry.update();
        }
        
        privateTelemetry.addData("heading", tempHeading);
        privateTelemetry.update();
        
        stop();
    }
    public void turnLeft(double speed, double degrees, double timeout ){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        double tempHeading = angles.firstAngle;
        
        runtime.reset();
        pivotLeft(speed);
        while (Math.abs(tempHeading - heading) <= -(degrees - offset) && runtime.seconds() < timeout) {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;
            double roll = angles.thirdAngle;
            double pitch = angles.secondAngle;
            privateTelemetry.addData("heading", heading);
            privateTelemetry.addData("roll", roll);
            privateTelemetry.addData("pitch", pitch);
            privateTelemetry.update();
        }
        
        if (runtime.seconds() >= timeout)
        {
            privateTelemetry.addData("timeOut","Completed!") ;
            privateTelemetry.update();
        }
        
        privateTelemetry.addData("heading", tempHeading);
        privateTelemetry.update();
        
        stop();
    }

    public void swivelRight(double speed, int gap , double timeout){
        runtime.reset();
        pivotRight(speed);
        mySleep(gap);
        stop();
        mySleep(250);
        pivotLeft(speed);
        mySleep(gap);
        stop();
        mySleep(250);
        stop();
    }
    public void swivelLeft(double speed, int gap, double timeout ){
        runtime.reset();
        pivotLeft(speed);
        mySleep(gap);
        stop();
        mySleep(250);
        pivotRight(speed);
        mySleep(gap);
        stop();
        mySleep(250);
        stop();
    }

}
