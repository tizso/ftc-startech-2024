package org.firstinspires.ftc.teamcode.startech;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "TeleOp", group = "StarTech")
public class TeleOpMode extends LinearOpMode {

    public DriveTrain driveTrain;
    @Override
    /*
     * Constructor for passing all the subsystems in order to make the subsystem be able to use
     * and work/be active
     */
    public void runOpMode() throws InterruptedException {
        /*SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);

        telemetry.clearAll();
        telemetry.addData("Running TeleOp","");
        telemetry.update();
        /* Wait for Start or Stop Button to be pressed */
        waitForStart();

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {
            while (opModeIsActive()) {
                driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
                driveTrain.gamepadInput = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
                driveTrain.gamepadInputTurn = -gamepad1.right_stick_x;
                driveTrain.driveTrainPointFieldModes();


                if(gamepad2.a){

                }

                if(gamepad2.y){

                }

                if(gamepad2.b){

                }

                //turn slider to right direction
                if(gamepad2.x){

                }

                if(gamepad2.left_bumper){

                }
                Pose2d poseEstimate = driveTrain.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.update();
            }
        }
    }
}