package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Config
public class AutoFluffy {
    LinearOpMode op;
    DcMotor liftMotor;
    Servo grabberRot, finger, hangerLatch, dronePusher, leftPurple, rightPurple;

    public MecanumDrive drive;

    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    //TfodProcessor tfod;
    RedFinder redFinder;
    final int RESOLUTION_WIDTH = 1920;
    final int RESOLUTION_HEIGHT = 1080;
    public static double DRONE_PUSHER_RESET = 0.85;
    public static double DRONE_PUSHER_INIT = 0.8;
    public static double GRABBER_ROT_INIT = 0.07;
    public static double GRABBER_UP = 0.3;
    public static double GRABBER_DOWN = GRABBER_ROT_INIT;
    public static double FINGER_UP = 0;
    public static double FINGER_DOWN = .4;
    public static double FINGER_INIT = FINGER_DOWN;

    public static double HANGER_LATCH_INIT = 0.87;

    public static double LEFT_PURPLE_RELEASE = .75;
    public static double LEFT_PURPLE_GRAB = .05;
    public static double RIGHT_PURPLE_RELEASE = 0;
    public static double RIGHT_PURPLE_GRAB = 1;
    public static double LEFT_PURPLE_INIT = LEFT_PURPLE_GRAB;
    public static double RIGHT_PURPLE_INIT = RIGHT_PURPLE_GRAB;
    public static int LIFT_UP = 0;  //fix values
    public static int LIFT_DOWN = 0;  //fix values
    public static double LIFT_POWER = 1;  //fix values

    boolean isGrabberUp = false;

    String side = "Red";

    String propLocation;


    String[] RED_LABELS = {"redprop"};
    String[] BLUE_LABELS = {"blueprop"};

    public AutoFluffy(LinearOpMode op) {
        this.op = op;
        this.init();
    }

    public AutoFluffy(LinearOpMode op, String side) {
        this.op = op;
        this.side = side;
        this.init();

    }

    /*public AutoFluffy() {

    }*/

    AprilTagDetection assignID (String propLocation, String side){
        int idNum=0;

        if (side == "Blue"){
            if (propLocation == "Left"){
                idNum=1;

            }else if (propLocation=="Center"){
                idNum = 2;
            }else if (propLocation=="Right"){
                idNum= 3;
            }
        }else if (side== "Red"){
            if (propLocation=="Left"){
                idNum= 4;
            }else if (propLocation=="Center"){
                idNum=5;
            }else if (propLocation=="Right"){
                idNum=6;
            }
        }
        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = findDetections();
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections){
            if (detection.id == idNum){
                return detection;
            }
        }
        return null;
    }


    public void telemetryDetection (AprilTagDetection detection){
        if (detection==null){
            return;
        }
        if (detection.metadata!= null){
            op.telemetry.addData("ID: ", detection.id);
            op.telemetry.addData("Range(Distance from board): ", detection.ftcPose.range);
            op.telemetry.addData("Yaw: ", detection.ftcPose.yaw);
            op.telemetry.addData("Bearing: ", detection.ftcPose.bearing);
        }else{
            op.telemetry.addData("ID: ", detection.id);
            return;
        }
        op.telemetry.update();
    }


    public void init() {
        liftMotor = op.hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        grabberRot= op.hardwareMap.servo.get("grabberRot");
        grabberRot.setPosition(GRABBER_ROT_INIT);

        dronePusher = op.hardwareMap.servo.get("dronePusher");
        dronePusher.setPosition(DRONE_PUSHER_INIT);

        finger = op.hardwareMap.servo.get("finger");
        finger.setPosition(FINGER_INIT);

        hangerLatch = op.hardwareMap.servo.get("hangerLatch");
        hangerLatch.setPosition(HANGER_LATCH_INIT);

        leftPurple = op.hardwareMap.servo.get("leftPurple");
        leftPurple.setPosition(LEFT_PURPLE_INIT);

        rightPurple = op.hardwareMap.servo.get("rightPurple");
        rightPurple.setPosition(RIGHT_PURPLE_INIT);

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        redFinder = new RedFinder();


        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------
        /*String[] LABELS;
        if (side.equals("Red")) {
            LABELS = RED_LABELS;
        } else {
            LABELS = BLUE_LABELS;
        }
        tfod = new TfodProcessor.Builder()
                .setModelFileName("model_20231209_112710.tflite")
                .setModelAspectRatio(RESOLUTION_WIDTH/RESOLUTION_HEIGHT)  //verify with grace
                .setModelLabels(LABELS)
                .build();
        */
        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------

        drive = new MecanumDrive(op.hardwareMap, new Pose2d(new Vector2d(0, 0), 0));

        visionPortal = new VisionPortal.Builder()
                .setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(redFinder, aprilTag)
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .build();

    }

    List<AprilTagDetection> findDetections() {
        return aprilTag.getDetections();

    }

    // JRC: Shouldn't "side" be "propLocation"?  "side" is reserved for "Red" and "Blue"
    public String getSide(){
        return redFinder.getSide();
    }

    // JRC: needs a getPropLocation() method

    public void deliverPurple() {
        leftPurple.setPosition(LEFT_PURPLE_RELEASE);
        rightPurple.setPosition(RIGHT_PURPLE_RELEASE);
        op.sleep(1000);
    }


    /*public List<Recognition> getRecognitions(){
        return tfod.getRecognitions();
    }*/

    public void raiseGrabber(){
        grabberRot.setPosition(GRABBER_UP);
        isGrabberUp=true;
    }


        public void lowerGrabber(){
            grabberRot.setPosition(GRABBER_DOWN);
            isGrabberUp=false;
        }

        public void setFingerUp(){finger.setPosition(FINGER_UP);
        }
        public void setFingerDown(){
            finger.setPosition(FINGER_DOWN);
        }
        public void raiseLift(){
        liftMotor.setTargetPosition(LIFT_UP);
        liftMotor.setPower(LIFT_POWER);
        }
        public void raiseFinger(){
        finger.setPosition(FINGER_UP);
        }
        public void grabberDown(){
        grabberRot.setPosition(GRABBER_DOWN);
        }
        public void lowerLift(){
        liftMotor.setTargetPosition(LIFT_DOWN);
        liftMotor.setPower(LIFT_POWER);
        }

}