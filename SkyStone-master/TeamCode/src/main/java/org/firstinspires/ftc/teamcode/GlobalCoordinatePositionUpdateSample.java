package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

/**
 * Created by Sarthak on 6/1/2019.
 * Example OpMode that runs the GlobalCoordinatePosition thread and accesses the (x, y, theta) coordinate values
 */
@TeleOp(name = "Global Coordinate Position Test", group = "Calibration")
public class GlobalCoordinatePositionUpdateSample extends LinearOpMode {

    //Odometry encoder wheels
    private SKYHardware SKY = new SKYHardware();


    @Override
    public void runOpMode() throws InterruptedException {
        SKY.init(hardwareMap);

        //Assign the hardware map to the odometry wheels
        SKY.verticalEncoderLeft = hardwareMap.dcMotor.get(SKY.verticalLeftEncoderName);
        SKY.verticalEncoderRight = hardwareMap.dcMotor.get(SKY.verticalRightEncoderName);
        SKY.horizontalEncoder = hardwareMap.dcMotor.get(SKY.horizontalEncoderName);

        //Reset the encoders
        SKY.verticalEncoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.verticalEncoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
        horizontal encoder travels to the right, it returns positive value
        */
        SKY.verticalEncoderRight.setDirection(DcMotorSimple.Direction.REVERSE);
        SKY.verticalEncoderLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        SKY.horizontalEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        SKY.verticalEncoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.verticalEncoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        /**
         * *****************
         * OpMode Begins Here
         * *****************
         */

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(SKY.verticalEncoderLeft, SKY.verticalEncoderRight, SKY.verticalEncoderLeft, SKY.COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / SKY.COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / SKY.COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }
}