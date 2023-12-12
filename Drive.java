package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether tLocalizationTesthe localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Drive")
public class Drive extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime();
    private DcMotor intakeMotor = null;

    private DcMotor liftMotor = null;

    private DcMotor slideMotor = null;

    private Servo firstLift = null;
    private CRServo secondLift = null;

    @Override
    public void runOpMode() throws InterruptedException {
        timer.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        firstLift = hardwareMap.get(Servo.class, "firstLift");
        secondLift = hardwareMap.get(CRServo.class, "secondLift");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );


            drive.update();

            //Slide lift code
            double slidePosition = slideMotor.getCurrentPosition();
            if (slidePosition > -100 && slidePosition < 4350) {
                slideMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            } else if (slidePosition < -100){
                slideMotor.setPower(1);
            } else if (slidePosition > 4350) {
                slideMotor.setPower(-1);
            }
            telemetry.addData("slidePosition", slidePosition);

            //intake Code
            if (gamepad1.b) {
                intakeMotor.setPower(1);
            } else if (gamepad1.a) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            // Climber Code
            if (gamepad2.right_bumper) {
                liftMotor.setPower(1);
            } else if (gamepad2.left_bumper) {
                liftMotor.setPower(-1);
            } else {
                liftMotor.setPower(0);
            }

            if (gamepad2.x) {
                firstLift.setPosition(1);
            } else if (gamepad2.y) {
                firstLift.setPosition(0);
            }

            if (gamepad2.a) {
                secondLift.setPower(1);
            } else if (gamepad2.b) {
                secondLift.setPower(-1);
            } else {
                secondLift.setPower(0);
            }




            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("timer", timer.time());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("intake", intakeMotor.getPower());
            telemetry.addData("Wench", liftMotor.getPower());
            telemetry.addData("slidePosition", slidePosition);
            telemetry.addData("Servo1", firstLift.getPosition());
            telemetry.update();


        }
    }
}
