package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class OdometryCalibration extends LinearOpMode {
    //Drive motors
    private SKYHardware SKY = new SKYHardware();
    //IMU Sensor
    BNO055IMU imu;

    ElapsedTime timer = new ElapsedTime();

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        SKY.init(hardwareMap);

        initHardwareMap(SKY.rfName, SKY.rbName, SKY.lfName, SKY.lbName, SKY.verticalLeftEncoderName, SKY.verticalRightEncoderName, SKY.horizontalEncoderName);
        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(getZAngle() < 90 && opModeIsActive()){
            SKY.rightF.setPower(-SKY.PIVOT_SPEED);
            SKY.rightB.setPower(-SKY.PIVOT_SPEED);
            SKY.leftF.setPower(SKY.PIVOT_SPEED);
            SKY.leftB.setPower(SKY.PIVOT_SPEED);
            if(getZAngle() < 60) {
                setPowerAll(-SKY.PIVOT_SPEED, -SKY.PIVOT_SPEED, SKY.PIVOT_SPEED, SKY.PIVOT_SPEED);
            }else{
                setPowerAll(-SKY.PIVOT_SPEED/2, -SKY.PIVOT_SPEED/2, SKY.PIVOT_SPEED/2, SKY.PIVOT_SPEED/2);
            }

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(SKY.verticalEncoderLeft.getCurrentPosition()) + (Math.abs(SKY.verticalEncoderRight.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*SKY.COUNTS_PER_INCH);

        SKY.horizontalTickOffset = SKY.horizontalEncoder.getCurrentPosition()/Math.toRadians(getZAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(SKY.horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", SKY.horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", -SKY.verticalEncoderLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", SKY.verticalEncoderRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", SKY.horizontalEncoder.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }

    private void initHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        SKY.rightF = hardwareMap.dcMotor.get(rfName);
        SKY.rightB = hardwareMap.dcMotor.get(rbName);
        SKY.leftF = hardwareMap.dcMotor.get(lfName);
        SKY.leftB= hardwareMap.dcMotor.get(lbName);

        SKY.verticalEncoderLeft = hardwareMap.dcMotor.get(vlEncoderName);
        SKY.verticalEncoderRight = hardwareMap.dcMotor.get(vrEncoderName);
        SKY.horizontalEncoder = hardwareMap.dcMotor.get(hEncoderName);

        SKY.rightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.rightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.leftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SKY.rightF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.rightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.leftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SKY.verticalEncoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.verticalEncoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SKY.verticalEncoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.verticalEncoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        SKY.rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SKY.rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SKY.leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SKY.leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //SKY.leftF.setDirection(DcMotorSimple.Direction.REVERSE);
        //SKY.rightF.setDirection(DcMotorSimple.Direction.REVERSE);
        //SKY.rightB.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();

    }

    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    private double getZAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }

    /**
     * Sets power to all four drive motors
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb){
        SKY.rightF.setPower(rf);
        SKY.rightB.setPower(rb);
        SKY.leftF.setPower(lf);
        SKY.leftB.setPower(lb);
    }

}
