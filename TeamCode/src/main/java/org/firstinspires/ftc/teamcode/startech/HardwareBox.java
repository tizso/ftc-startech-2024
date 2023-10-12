package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left Front motor:        "left_front"
 * Motor channel:  Left Rear motor:         "left_rear"
 * Motor channel:  Right Front motor:       "right_front"
 * Motor channel:  Right Rear motor:        "right_rear"
 * Motor channel:  Carouser drive motor:    "carouser"
 * Motor channel:  Arm motor:               "arm"
 *
 * Servo channel:  Servo to open claw:	   "claw"
 * Servo channel:  Servo to move arm:		"arm"
 * Servo channel:  Servo to move platform:   "platform"
 * Servo channel:  Servo to move adaugare:   "adaugare"
 * Sensor channel: Color sensor:			 "color"
 *
 */
public class HardwareBox
{
    /* Public OpMode members. */
    public DcMotor  leftFront   = null;
    public DcMotor  leftRear	= null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightRear   = null;
    //public DcMotor  carouser	= null;
    //public DcMotor  arm         = null;
    //public DcMotor  slider      = null;
    //public Servo  claw		= null;



    public static final double  VELOCITY               = 200;
    public static final int  TARGET_POS                = 250;
    public static final int  LEVEL_POS                 = 100;

    /* local OpMode members. */
    DigitalChannel touchSensor;
    HardwareMap hwMap		   =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBox(){

    }



    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront   = hwMap.get(DcMotor.class, "leftFront");
        leftRear    = hwMap.get(DcMotor.class, "leftRear");
        rightFront  = hwMap.get(DcMotor.class, "rightFront");
        rightRear   = hwMap.get(DcMotor.class, "rightRear");
        //carouser    = hwMap.get(DcMotor.class, "carouser");
        //arm         = hwMap.get(DcMotorEx.class, "arm");
        //slider	    = hwMap.get(DcMotor.class, "slider");

        // Set all motors to zero power
        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);
        //carouser.setPower(0.0);
        //arm.setPower(0.0);
        //slider.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //carouser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize ALL installed servos.
        //claw  = hwMap.get(Servo.class, "claw");
        //claw.setPosition(0);


    }

    public void forward(){
        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.FORWARD);
    };

    public void back(){
        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.REVERSE);
    }

    public void left(){
        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.REVERSE);
    };

    public void right(){
        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.FORWARD);
    };

    public void turnLeft(){
        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.FORWARD);
    };

    public void turnRight(){
        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.REVERSE);
    };
}