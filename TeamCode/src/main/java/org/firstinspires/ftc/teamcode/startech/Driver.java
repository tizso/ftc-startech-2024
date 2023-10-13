package org.firstinspires.ftc.teamcode.startech;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareBox;

@Disabled
@TeleOp(name="DriverMode", group="StarTech")

public class Driver extends LinearOpMode {
    HardwareBox robot = new org.firstinspires.ftc.teamcode.HardwareBox();

    @Override
    public void runOpMode() {

        waitForStart();
        robot.init(hardwareMap);
        if(opModeIsActive()){


            while (opModeIsActive()){
                double leftFrontPower;
                double leftBackPower;
                double rightFrontPower;
                double rightBackPower;

                double drive = gamepad1.left_stick_y;
                double turn  =  gamepad1.left_stick_x;

                robot.back();

                leftFrontPower    = Range.clip(drive - turn, -1.0, 1.0);
                leftBackPower    = Range.clip(drive + turn, -1.0, 1.0);
                rightFrontPower   = Range.clip(drive + turn, -1.0, 1.0);
                rightBackPower   = Range.clip(drive - turn, -1.0, 1.0);

                if(gamepad1.left_bumper) {
                    robot.turnLeft();
                    leftFrontPower = 0.0;
                    leftBackPower = 0.8;
                    rightFrontPower = 0.8;
                    rightBackPower = 0.0;

                }
                if(gamepad1.right_bumper) {
                    robot.turnRight();
                    leftFrontPower = 0.8;
                    leftBackPower = 0;
                    rightFrontPower = 0;
                    rightBackPower = 0.8;
                }
                if (gamepad1.a) {

                }

                if(gamepad1.x){

                }
                if (gamepad1.b) {

                }
                if (gamepad1.y) {

                }

                //double armSpeed = 0.5;

                if(gamepad2.a){

                }

                if(gamepad2.x){

                }
                if(gamepad2.y){

                }

                if(gamepad2.left_bumper){

                }

                robot.leftFront.setPower(leftFrontPower);
                robot.leftRear.setPower(leftBackPower);
                robot.rightFront.setPower(rightFrontPower);
                robot.rightRear.setPower(rightBackPower);

                telemetry.addData("Motors", "leftFront (%.2f), leftBack (%.2f), rightFront (%.2f), rightBack (%.2f)",
                        leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);

                telemetry.update();
            }
        }

    }
}
