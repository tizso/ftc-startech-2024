package org.firstinspires.ftc.teamcode.startech;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "AutoRedLeft", group = "StarTech")
public class AutoRedLeft extends LinearOpMode{


    static final double FEET_PER_METER = 3.28084;


    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public DriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {



        /*Create your Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);

        //Key Pay inputs to selecting Starting Position of robot
        startPosition = START_POSITION.RED_RIGHT;
        //selectStartingPosition();


        telemetry.setMsTransmissionInterval(50);
        //Build Autonomous trajectory to be used based on starting position selected
        buildAuto();
        driveTrain.getLocalizer().setPoseEstimate(initPose);

        while (!isStopRequested() && !opModeIsActive()) {
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.

            telemetry.update();
            sleep(20);
        }


        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Stop Vision process

            //Build parking trajectory based on last detected target by vision
            buildParking();

            //run Autonomous trajectory
            runAutoAndParking();
        }

        //Trajectory is completed, display Parking complete
        parkingComplete();
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    }


    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryAuto, trajectoryParking ;

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    Pose2d midWayPose;
    Pose2d startParkPose;

    Pose2d dropConePose0;
    Pose2d pickConePose1;
    Pose2d pickConePose2;
    Pose2d dropConePose1;

    Pose2d parkPose;

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        initPose = new Pose2d(63, -36, Math.toRadians(180));//Starting pose
        midWayPose = new Pose2d(3, -36, Math.toRadians(180)); //partea rosie
        startParkPose = new Pose2d(8, -35, Math.toRadians(180)); //Choose the pose to move forward towards signal cone

        dropConePose0 = new Pose2d(4, -35, Math.toRadians(122)); //Choose the pose to move to the stack of cones
        pickConePose1 = new Pose2d(5, -43.5, Math.toRadians(74)); //Choose the pose to move to the stack of cones
        pickConePose2 = new Pose2d(5    , -42.6, Math.toRadians(74)); //Choose the pose to move to the stack of cones
        dropConePose1 = new Pose2d(4, -34, Math.toRadians(137)); //Choose the pose to move to the stack of cones

        //Drop Preloaded Cone, Pick 5 cones and park
        trajectoryAuto = driveTrain.trajectorySequenceBuilder(initPose)
                //New autonom
                .lineToLinearHeading(midWayPose)
                //Drop preloaded cone
                .lineToLinearHeading(dropConePose0)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () ->{

                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () ->{
                    setSliderPos(2600, 1, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, ()->{
                    setSliderPos(0, 1, 0);
                })
                .waitSeconds(1.8)
                //Pick up 1 cone
                .lineToLinearHeading(pickConePose1)
                .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {

                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {

                })

                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {

                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {

                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {

                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {

                })
                .waitSeconds(1.8)

                //Drop 1 cone
                .lineToLinearHeading(dropConePose1)
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () ->{

                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () ->{
                    setSliderPos(2600, 1, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, ()->{
                    setSliderPos(0, 1, 0);
                })
                .waitSeconds(1.8)

                //Pick 2 cone
                .lineToLinearHeading(pickConePose1)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {

                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {

                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {

                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {

                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {

                })
                .UNSTABLE_addTemporalMarkerOffset(1.8, () -> {

                })
                .waitSeconds(2.1)
                .lineToLinearHeading(dropConePose1)

                //Drop 2 cone
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () ->{

                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () ->{
                    setSliderPos(2600, 1, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, ()->{
                    setSliderPos(0, 1, 0);
                })
                .waitSeconds(1.8)

                .lineToLinearHeading(startParkPose)
                .build();
    }

    //Build parking trajectory based on target detected by vision
    public void buildParking(){
        parkPose = new Pose2d(10, -74, Math.toRadians(180));

        trajectoryParking = driveTrain.trajectorySequenceBuilder(startParkPose)
                .lineToLinearHeading(parkPose)
                .build();
    }

    //Run Auto trajectory and parking trajectory
    public void runAutoAndParking(){
        telemetry.setAutoClear(false);
        telemetry.addData("Running Autonomous Mode","");
        telemetry.addData("---------------------------------------","");
        telemetry.update();
        //Run the trajectory built for Auto and Parking
        driveTrain.followTrajectorySequence(trajectoryAuto);
        driveTrain.followTrajectorySequence(trajectoryParking);
    }

    public void setSliderPos(int pos, double speed, int dir){
        telemetry.addData("pos:", pos);
        telemetry.addData("speed", speed);

    }

    //Write a method which is able to pick the cone from the stack depending on your subsystems


    public void parkingComplete(){
        telemetry.addData("Parked in Location","");
        telemetry.update();
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing Autonomous Mode","");
            telemetry.addData("---------------------------------------","");

            startPosition = START_POSITION.RED_RIGHT;
            telemetry.update();
        }
        telemetry.clearAll();
    }
}