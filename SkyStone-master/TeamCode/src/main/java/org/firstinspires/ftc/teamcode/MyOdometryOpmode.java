package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {

    private SKYHardware SKY = new SKYHardware();

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        SKY.init(hardwareMap);
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(SKY.rfName, SKY.rbName, SKY.lfName, SKY.lbName, SKY.verticalLeftEncoderName, SKY.verticalRightEncoderName, SKY.horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(SKY.verticalEncoderLeft, SKY.verticalEncoderRight, SKY.horizontalEncoder, SKY.COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / SKY.COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / SKY.COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", SKY.verticalEncoderLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", SKY.verticalEncoderRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", SKY.horizontalEncoder.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }
    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while(opModeIsActive() && distance > allowableDistanceError){
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            moveToPosition(robot_movement_x_component, robot_movement_y_component, pivotCorrection);
        }
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        SKY.rightF = hardwareMap.dcMotor.get(rfName);
        SKY.rightB = hardwareMap.dcMotor.get(rbName);
        SKY.leftF = hardwareMap.dcMotor.get(lfName);
        SKY.leftF = hardwareMap.dcMotor.get(lbName);

        SKY.verticalEncoderLeft = hardwareMap.dcMotor.get(vlEncoderName);
        SKY.verticalEncoderRight = hardwareMap.dcMotor.get(vrEncoderName);
        SKY.horizontalEncoder = hardwareMap.dcMotor.get(hEncoderName);

        SKY.rightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.rightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SKY.rightF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.rightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SKY.verticalEncoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.verticalEncoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SKY.horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SKY.verticalEncoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.verticalEncoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SKY.horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SKY.rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SKY.rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SKY.leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SKY.leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        //right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    private void moveToPosition(double robot_move_x, double robot_move_y, double robot_pivot){
        double frontLeftPower = (robot_move_y) + robot_move_x + (robot_pivot);
        double backLeftPower = (robot_move_y) - robot_move_x - (robot_pivot);
        double frontRightPower = (robot_move_y) - robot_move_x + (robot_pivot);
        double backRightPower = (robot_move_y) + robot_move_x - (robot_pivot);
        SKY.leftF.setPower(frontLeftPower);
        SKY.leftB.setPower(backLeftPower);
        SKY.rightF.setPower(frontRightPower);
        SKY.rightB.setPower(backRightPower);
        telemetry.addData("Left Front Power", frontLeftPower);
        telemetry.addData("Left Rear Power", backLeftPower);
        telemetry.addData("Right Front Power", frontRightPower);
        telemetry.addData("Right Rear Power", backRightPower);
    }
}
