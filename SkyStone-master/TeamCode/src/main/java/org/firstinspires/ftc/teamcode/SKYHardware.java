package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SKYHardware {

    // Hardware Map
    HardwareMap skyhw = null;

    // DC motors
    public DcMotor leftF, leftB, rightF, rightB; // drive motors
    public DcMotor armV, armH; // vertical and horizontal stack arms
    public DcMotor armB1, armB2; // rear horizontal rack arms
    public DcMotor verticalEncoderLeft, verticalEncoderRight, horizontalEncoder; //odometry wheels

    // hardware map strings for encoders and DcDrive motors
    String rfName = "rightF", rbName = "rightB", lfName = "leftF", lbName = "leftB";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = lbName;
    /*
    NOTE: The above code determines which motor ports the encoders actually go in:
    FROM THE PERSPECTIVE FROM THE REAR OF THE ROBOT (auto arm side):
    verticalLeftEncoder = leftF (port 0)
    verticalRightEncoder = rightF (port 2)
    horizontalEncoder = leftB (port 1)
     */

    // servos
    public Servo grabRight, grabLeft; // front block grabbers
    public Servo grabHolder; // rear buildplate servo
    public Servo grabLeftF, grabRightF; // front buildplate servos
    public CRServo parkServo; // tape measure Vex Motors

    //odometer values
    final double PIVOT_SPEED = 0.5; // the speed that the robot turns during odometry calibration
    final double COUNTS_PER_INCH = 651.899; // amount of ticks in 1 inch for odometers
    double horizontalTickOffset = 0; // tick offset

    final String vuforiaKey = "AQMXapn/////AAABmUkIKliu8UhAlsC0hI8AZ/gGzHN693N16RDIV6KvJMcygzolaMYuceUhBHEFuw9JwHBpBSS2OV/BEczUwrgYp9iMPev1ooBl10M89qxmmps38aXL7YycUEe3FTH/0YnvFmPCqUc60Hr0rpAgYqcbmKNfGPF7GCVYsHDGTjUUJAepX5HiX1UUES01Wji5ZArDu9A3oTSMvjSVULFB6wLXRKK8Qk8p/sh3NZsg11NtgjePsUckyvJXTVxTaRwltAWBh9eLZsMwHsZD5pcUSsJwXQFIqGwYE7T7fTMGhPZw/V1bsKTzp7rw5ErPbeBvLUzyHe9DlIyLbJqQ1pIoF9UP+PbQgz3HHf0F7bsKpc3EGa0l";

    public SKYHardware() { } // default constructor

    public void init(HardwareMap sshw) {
        leftB = sshw.dcMotor.get(lbName);
        rightB = sshw.dcMotor.get(rbName);
        rightF = sshw.dcMotor.get(rfName);
        leftF = sshw.dcMotor.get(lfName);
        armV = sshw.dcMotor.get("armV");
        armH = sshw.dcMotor.get("armH");
        armB1 = sshw.dcMotor.get("armB1");
        armB2 = sshw.dcMotor.get("armB2");
        grabRight = sshw.servo.get("grabRight");
        grabLeft = sshw.servo.get("grabLeft");
        grabHolder = sshw.servo.get("grabHolder");
        grabLeftF = sshw.servo.get("grabLeftF");
        grabRightF = sshw.servo.get("grabRightF");
        parkServo = sshw.crservo.get("parkServo");
        rightF.setDirection(DcMotor.Direction.REVERSE);
        leftB.setDirection(DcMotor.Direction.REVERSE);
        armB2.setDirection(DcMotor.Direction.REVERSE);
        armV.setDirection(DcMotor.Direction.REVERSE);
        armV.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}