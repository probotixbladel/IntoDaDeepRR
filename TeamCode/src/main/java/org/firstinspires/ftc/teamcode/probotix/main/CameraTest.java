package org.firstinspires.ftc.teamcode.probotix.main;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Disabled
@TeleOp
public class CameraTest extends LinearOpMode {
    private hardware Hardware;
    OpenCvWebcam webcam;
    redPropPipeline pipeline;

    public void runOpMode() throws InterruptedException {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new redPropPipeline();
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
            telemetry.addData("Y Location", pipeline.getY());
            telemetry.update();

            propXPos = pipeline.getX();

            sleep(50);
            if (propXPos < 420) {
                trajNumber = 1;
            } else if (propXPos > 420) {
                if (propXPos < 850) {
                    trajNumber = 2;
                } else if (propXPos > 850) {
                    trajNumber = 3;
                }
            }
        }
        if (propXPos < 420) {
            trajNumber = 1;
        } else if (propXPos > 420) {
            if (propXPos < 850) {
                trajNumber = 2;
            } else if (propXPos > 850) {
                trajNumber = 3;
            }

        }
        telemetry.addData("traj:", trajNumber);

    }
}

