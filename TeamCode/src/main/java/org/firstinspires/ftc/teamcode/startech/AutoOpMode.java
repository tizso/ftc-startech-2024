package org.firstinspires.ftc.teamcode.startech;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
@Disabled
@Autonomous(name = "Autonomous", group = "StarTech")
public class AutoOpMode extends LinearOpMode{
    private DcMotor slider;
    public Servo claw = null;
    public Servo clawRot = null;
    public Servo armL = null;
    public Servo armR = null;
    public Servo armSlidL = null;
    public Servo armSlidR = null;
    public Servo armSlider = null;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 946.461;
    double fy = 946.136;
    double cx = 312.211;
    double cy = 211.465;

    double tagsize = 0.166;

    int left = 10;
    int middle = 11;
    int right = 12;

    AprilTagDetection tagOfInterest = null;

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public DriveTrain driveTrain;
    int sliderPos = 0;
    int clawPos = 0;
    double sliderSpeed = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        slider = hardwareMap.get(DcMotor.class, "slider");
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        claw  = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(0.5);

        clawRot  = hardwareMap.get(Servo.class, "clawRot");
        clawRot.setPosition(0);

        armL  = hardwareMap.get(Servo.class, "armL");
        armL.setPosition(0.3);

        armR  = hardwareMap.get(Servo.class, "armR");
        armR.setPosition(0.7);

        armSlidL  = hardwareMap.get(Servo.class, "armSlidL");
        armSlidL.setPosition(0.9);

        armSlidR  = hardwareMap.get(Servo.class, "armSlidR");
        armSlidR.setPosition(0.1);

        armSlider  = hardwareMap.get(Servo.class, "armSlider");
        armSlider.setPosition(1);

        /*Create your Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        //Build Autonomous trajectory to be used based on starting position selected
        buildAuto();
        driveTrain.getLocalizer().setPoseEstimate(initPose);

        while (!isStopRequested() && !opModeIsActive()) {
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle || tag.id == right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            telemetry.update();
            sleep(20);
        }

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if(tagOfInterest == null || tagOfInterest.id == left){

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
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(63, -36, Math.toRadians(180));//Starting pose
                midWayPose = new Pose2d(5, -36, Math.toRadians(180)); //Partea albastru
                startParkPose = new Pose2d(5, -35, Math.toRadians(180)); //Choose the pose to move forward towards signal cone

                dropConePose0 = new Pose2d(3, -35, Math.toRadians(122)); //Choose the pose to move to the stack of cones
                pickConePose1 = new Pose2d(4, -44, Math.toRadians(75)); //Choose the pose to move to the stack of cones
                pickConePose2 = new Pose2d(4, -44, Math.toRadians(75)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(4, -34, Math.toRadians(138)); //Choose the pose to move to the stack of cones
                break;
            case RED_LEFT:
                initPose = new Pose2d(63, -36, Math.toRadians(180));//Starting pose
                midWayPose = new Pose2d(13, -36, Math.toRadians(180)); //partea rosie
                startParkPose = new Pose2d(5, -35, Math.toRadians(180)); //Choose the pose to move forward towards signal cone

                dropConePose0 = new Pose2d(5, -35, Math.toRadians(122)); //Choose the pose to move to the stack of cones
                pickConePose1 = new Pose2d(4, -43.9, Math.toRadians(76)); //Choose the pose to move to the stack of cones
                pickConePose2 = new Pose2d(4, -44.0, Math.toRadians(76)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(4, -34, Math.toRadians(138)); //Choose the pose to move to the stack of cones

                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(63, 36, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(8.5, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                startParkPose = new Pose2d(3, 35, Math.toRadians(180)); //Choose the pose to move forward towards signal cone

                dropConePose0 = new Pose2d(3, 35, Math.toRadians(245)); //Choose the pose to move to the stack of cones
                pickConePose1 = new Pose2d(4, 43.9, Math.toRadians(290));
                pickConePose2 = new Pose2d(4, 44.2, Math.toRadians(290)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(4, 34, Math.toRadians(225));
            case RED_RIGHT:
                initPose = new Pose2d(63, 36, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(9, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                startParkPose = new Pose2d(6, 35, Math.toRadians(180)); //Choose the pose to move forward towards signal cone

                dropConePose0 = new Pose2d(3, 35, Math.toRadians(245)); //Choose the pose to move to the stack of cones
                pickConePose1 = new Pose2d(4, 44.4, Math.toRadians(294));
                pickConePose2 = new Pose2d(4, 44.5, Math.toRadians(293)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(4, 35, Math.toRadians(225));

                break;
        }

        //Drop Preloaded Cone, Pick 5 cones and park
        trajectoryAuto = driveTrain.trajectorySequenceBuilder(initPose)
                //New autonom
                .lineToLinearHeading(midWayPose)
                //Drop preloaded cone
                .lineToLinearHeading(dropConePose0)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () ->{
                    claw.setPosition(0.5);
                    armL.setPosition(0.3);
                    armR.setPosition(0.7);
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
                    armL.setPosition(1.0);
                    armR.setPosition(0.0);
                    clawRot.setPosition(0.2);
                    armSlidL.setPosition(0.1);
                    armSlidR.setPosition(0.9);
                    claw.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    armSlider.setPosition(0);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    claw.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    armL.setPosition(0.2);
                    armR.setPosition(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    armSlider.setPosition(0.9);
                    clawRot.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    armL.setPosition(0.0);
                    armR.setPosition(1.0);
                })
                .waitSeconds(1.8)

                //Drop 1 cone
                .lineToLinearHeading(dropConePose1)
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () ->{
                    claw.setPosition(0.5);
                    armL.setPosition(0.4);
                    armR.setPosition(0.6);
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
                    armL.setPosition(1.0);
                    armR.setPosition(0.0);
                    clawRot.setPosition(0.2);
                    claw.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armSlidL.setPosition(0.4);
                    armSlidR.setPosition(0.6);
                    armSlider.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    claw.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    armL.setPosition(0.5);
                    armR.setPosition(0.5);
                    armSlidL.setPosition(0.4);
                    armSlidR.setPosition(0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    armSlider.setPosition(0.9);
                    clawRot.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.8, () -> {
                    armL.setPosition(0.0);
                    armR.setPosition(1.0);
                })
                .waitSeconds(2.1)
                .lineToLinearHeading(dropConePose1)

                //Drop 2 cone
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () ->{
                    claw.setPosition(0.5);
                    armL.setPosition(0.4);
                    armR.setPosition(0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () ->{
                    setSliderPos(2600, 1, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, ()->{
                    setSliderPos(0, 1, 0);
                })
                .waitSeconds(1.8)

                //pick 3
                .lineToLinearHeading(pickConePose2)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    armL.setPosition(1.0);
                    armR.setPosition(0.0);
                    clawRot.setPosition(0.2);
                    claw.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    armSlidL.setPosition(0.5);
                    armSlidR.setPosition(0.5);
                    armSlider.setPosition(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    claw.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                    armL.setPosition(0.5);
                    armR.setPosition(0.5);
                    armSlidL.setPosition(0.4);
                    armSlidR.setPosition(0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    armSlider.setPosition(0.9);
                    clawRot.setPosition(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.9, () -> {
                    armL.setPosition(0.0);
                    armR.setPosition(1.0);
                })
                .waitSeconds(2.3)

                //Drop cone 3
                .lineToLinearHeading(dropConePose1)
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () ->{
                    claw.setPosition(0.5);
                    armL.setPosition(0.4);
                    armR.setPosition(0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () ->{
                    setSliderPos(2600, 1, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, ()->{
                    setSliderPos(0, 1, 0);
                    claw.setPosition(1.0);
                })
                .waitSeconds(1.8)

                .lineToLinearHeading(startParkPose)
                .build();
    }

    //Build parking trajectory based on target detected by vision
    public void buildParking(){
        switch (startPosition) {

            case BLUE_LEFT:
                switch(tagOfInterest.id){
                    case 10: parkPose = new Pose2d(10, -74, Math.toRadians(180)); break; // Location 1
                    //case 11: parkPose = new Pose2d(10, -35, Math.toRadians(180)); break; // Location 2
                    case 11: parkPose = new Pose2d(5.5, -39, Math.toRadians(180)); break; // Location 2
                    case 12: parkPose = new Pose2d(4.5, -1, Math.toRadians(180)); break; // Albastru
                }
                break;
            case RED_LEFT:
                switch(tagOfInterest.id){
                    case 10: parkPose = new Pose2d(10, -74, Math.toRadians(180)); break; // Location 1
                    //case 11: parkPose = new Pose2d(10, -35, Math.toRadians(180)); break; // Location 2
                    case 11: parkPose = new Pose2d(4, -35, Math.toRadians(180)); break; // Location 2
                    case 12: parkPose = new Pose2d(10, -3, Math.toRadians(180)); break; // Rosu
                }
                break;
            case BLUE_RIGHT:
                switch(tagOfInterest.id){
                    case 10: parkPose = new Pose2d(5, 2, Math.toRadians(180)); break; // Location 1
                    case 11: parkPose = new Pose2d(5.1, 41, Math.toRadians(180)); break; // Location 2
                    case 12: parkPose = new Pose2d(6, 74, Math.toRadians(180)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                switch(tagOfInterest.id){
                    case 10: parkPose = new Pose2d(4.5, 1.5, Math.toRadians(180)); break; // Location 1
                    case 11: parkPose = new Pose2d(5.1, 40, Math.toRadians(180)); break; // Location 2
                    case 12: parkPose = new Pose2d(5.5, 74, Math.toRadians(180)); break; // Location 3
                }
                break;
        }

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
        if(dir==1){
            slider.setDirection(DcMotor.Direction.REVERSE);
        } else {
            slider.setDirection(DcMotor.Direction.FORWARD);
        }
        slider.setTargetPosition(pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(speed);
    }

    //Write a method which is able to pick the cone from the stack depending on your subsystems


    public void parkingComplete(){
        telemetry.addData("Parked in Location", tagOfInterest.id);
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
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }
}