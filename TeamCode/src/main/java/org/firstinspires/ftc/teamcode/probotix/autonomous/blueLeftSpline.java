package org.firstinspires.ftc.teamcode.probotix.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.probotix.main.bluePropPipeline;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="blueLeftSpline", group="probotix")
public class blueLeftSpline extends LinearOpMode {
    private hardware Hardware;
    OpenCvWebcam webcam;
    bluePropPipeline pipeline;
    public void runOpMode() throws InterruptedException {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new bluePropPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        int propXPos = 0;
        int trajNumber = 0;
        while (!isStarted() && !isStopRequested()) {

            telemetry.addData("X Location: ", pipeline.getX());
            //telemetry.addData("Y Location", pipeline.getY());
            telemetry.update();

            propXPos = pipeline.getX();

            sleep(50);
            if (propXPos<420){
                trajNumber = 1;
            }
            else if (propXPos>420){
                if (propXPos<850){
                    trajNumber =2;
                }
                else if(propXPos>850){
                    trajNumber = 3;
                }
            }
        }
        if (propXPos<420){
            trajNumber = 1;
        }
        else if (propXPos>420){
            if (propXPos<850){
                trajNumber =2;
            }
            else if(propXPos>850){
                trajNumber = 3;
            }
            telemetry.addData("traj:",trajNumber);
        }



        this.Hardware = new hardware(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //forward -x, backward +x, left -y, right +y
        // We want to start the bot at x: 0, y: 0, heading: 0 degrees
        //Pose2d startPose = new Pose2d(62, 35, Math.toRadians(90));
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        //******************************** LEFT ******************************//

        TrajectorySequence deliverleft = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-30,-11), Math.toRadians(0))
                .build();
        TrajectorySequence backupLeft = drive.trajectorySequenceBuilder(deliverleft.end())
                .splineToConstantHeading(new Vector2d(-20, -13), Math.toRadians(0))
                .build();

        TrajectorySequence deliverBackdropLeft = drive.trajectorySequenceBuilder(backupLeft.end())
                .lineToLinearHeading(new Pose2d(-22, -41, Math.toRadians(90)))
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(deliverBackdropLeft.end())
                .splineToConstantHeading(new Vector2d(-5, -37), Math.toRadians(0))
                .build();

        //******************************** LEFT ******************************//

        //******************************** MIDDLE ****************************//

        TrajectorySequence middle = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoClose))
                .waitSeconds(0.2)
                .lineTo(new Vector2d(-33, 0))
                .lineTo(new Vector2d(-31, 0))
                .addTemporalMarker(() -> Hardware.dropServo.setPosition(DriveConstants.dropServoOpen))
                .waitSeconds(0.3)
                .lineTo(new Vector2d(-20,0))
                .lineToLinearHeading(new Pose2d(-23, -41, Math.toRadians(90)))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(DriveConstants.armDeliver))
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver))
                .addTemporalMarker(() -> Hardware.dropServo.setPosition(DriveConstants.dropServoClose))
                .waitSeconds(1)
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoOpen))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown+295))
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(1650))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoPickUp))
                .splineTo(new Vector2d(-55,-20),Math.toRadians(90))
                .lineTo(new Vector2d(-49, 64))
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoClose))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(-55, -20))
                .lineToLinearHeading(new Pose2d(-30,-41,Math.toRadians(90)))
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(DriveConstants.liftDown))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(DriveConstants.armDeliver))
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver))
                .addTemporalMarker(() -> Hardware.dropServo.setPosition(DriveConstants.dropServoClose))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> Hardware.grabServo.setPosition(DriveConstants.grabServoOpen))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> Hardware.armMotor.setTargetPosition(0))
                .addTemporalMarker(() -> Hardware.liftMotor.setTargetPosition(10))
                .addTemporalMarker(() -> Hardware.flipServo.setPosition(DriveConstants.flipServoInit))
                .waitSeconds(0.5)
                .build();

        //******************************** MIDDLE ****************************//

        //**************************** RIGHT *************************************//

        TrajectorySequence deliverRight = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-28,0))
                .build();

        TrajectorySequence backupRight = drive.trajectorySequenceBuilder(deliverRight.end())
                .lineToLinearHeading(new Pose2d(-29,5, Math.toRadians(-90)))
                .build();

        TrajectorySequence deliverBackdropRight = drive.trajectorySequenceBuilder(backupRight.end())
                .lineToLinearHeading(new Pose2d(-33, -40, Math.toRadians(90)))
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(deliverBackdropRight.end())
                .lineToConstantHeading(new Vector2d(-50, -37))
                .build();

        //**************************** RIGHT *************************************//


        waitForStart();
        webcam.stopStreaming();
        if (!isStopRequested()) {
            if(trajNumber == 1){
                Hardware.armMotor.setPower(1);
                Hardware.liftMotor.setPower(1);
                Hardware.grabServo.setPosition(DriveConstants.grabServoClose);
                sleep(10);
                drive.followTrajectorySequence(deliverleft);
                Hardware.dropServo.setPosition(DriveConstants.dropServoOpen);
                sleep(500);
                drive.followTrajectorySequence(backupLeft);
                Hardware.dropServo.setPosition(DriveConstants.dropServoClose);
                drive.followTrajectorySequence(deliverBackdropLeft);
                Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp);
                Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver);
                sleep(500);
                Hardware.grabServo.setPosition(DriveConstants.grabServoOpen);
                sleep(700);
                Hardware.armMotor.setTargetPosition(0);
                sleep(100);
                Hardware.liftMotor.setTargetPosition(10);
                Hardware.flipServo.setPosition(DriveConstants.flipServoInit);
                sleep(200);
                drive.followTrajectorySequence(parkLeft);

            }
            else if(trajNumber == 2){
                Hardware.armMotor.setPower(1);
                Hardware.liftMotor.setPower(1);
                drive.followTrajectorySequence(middle);

            }
            else{
                Hardware.armMotor.setPower(1);
                Hardware.liftMotor.setPower(1);
                Hardware.grabServo.setPosition(DriveConstants.grabServoClose);
                sleep(10);
                drive.followTrajectorySequence(deliverRight);
                drive.followTrajectorySequence(backupRight);
                Hardware.dropServo.setPosition(DriveConstants.dropServoOpen);
                sleep(500);
                drive.followTrajectorySequence(deliverBackdropRight);
                Hardware.dropServo.setPosition(DriveConstants.dropServoClose);
                Hardware.armMotor.setTargetPosition(DriveConstants.armDelAuto);
                Hardware.liftMotor.setTargetPosition(DriveConstants.liftUp);
                Hardware.flipServo.setPosition(DriveConstants.flipServoDeliver);
                sleep(500);
                Hardware.grabServo.setPosition(DriveConstants.grabServoOpen);
                sleep(700);
                Hardware.armMotor.setTargetPosition(0);
                sleep(100);
                Hardware.liftMotor.setTargetPosition(10);
                Hardware.flipServo.setPosition(DriveConstants.flipServoInit);
                sleep(200);
                drive.followTrajectorySequence(parkRight);

            }
        }
    }
}
