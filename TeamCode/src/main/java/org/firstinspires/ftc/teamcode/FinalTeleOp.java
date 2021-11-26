package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class FinalTeleOp extends LinearOpMode {

    // Declaring Motors & Servos
    private DcMotorEx leftFront;        //port 0
    private DcMotorEx rightFront;       //port 1
    private DcMotorEx leftBack;         //port 1
    private DcMotorEx rightBack;        //port 3
    private DcMotor spinner;
    private DcMotor arm;
    private Servo wrist;        //port 0
    private Servo fingers;         //port 0

    //Declaring Distance Sensor Variables
    private DistanceSensor leftDistance;
    private DistanceSensor rightDistance;

    //Declaring Color Sensor Variables
    NormalizedColorSensor lfColorSensor;
    NormalizedColorSensor rfColorSensor;
    NormalizedRGBA colors;
    boolean foundRed = false;
    boolean foundWhite = false;

    //Declaring IMU Variables
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double heading;
    boolean turned = false;

    //Declaring Camera Variables
    OpenCvCamera webcam;
    OpenCVTest.SamplePipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0) {
                leftFront.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
                rightFront.setPower(-gamepad1.left_stick_y-gamepad1.right_stick_x);
                leftBack.setPower(-gamepad1.left_stick_y+gamepad1.right_stick_x);
                rightBack.setPower(-gamepad1.left_stick_y-gamepad1.right_stick_x);
            } else {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }
            if (gamepad1.a) {
                spinner.setPower(-0.55);
            } else if (gamepad1.b) {
                spinner.setPower(0.55);
            } else {
                spinner.setPower(0);
            }

            //Arm Raise and Lower
            if (gamepad2.right_stick_y != 0) {
                arm.setPower(-gamepad2.right_stick_y);
            } else {
                arm.setPower(0);
            }

            //Spin Spinner
            if (gamepad2.a) {
                spinner.setPower(-0.55);
            } else if (gamepad2.b) {
                spinner.setPower(0.55);
            } else {
                spinner.setPower(0);
            }

            //Open and Close Fingers
            if (gamepad2.dpad_up) {
                fingers.setPosition(.5);
            } else if (gamepad2.dpad_down) {
                fingers.setPosition(0.1);
            }

            //Turn Wrist In and Out
            if (gamepad2.left_bumper) {
                wrist.setPosition(.25);
            } else if (gamepad2.right_bumper) {
                wrist.setPosition(-.25);
            }

        }}

    public void stopRobot(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        spinner.setPower(0);
        arm.setPower(0);
    }

    public void initialize() {
        telemetry.addData("Stat", "Initializing...");
        telemetry.update();

        //Mapping Motors
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        spinner = hardwareMap.dcMotor.get("spinner");
        arm = hardwareMap.dcMotor.get("arm");

        //Mapping Servos
        wrist = hardwareMap.servo.get("wrist");
        fingers = hardwareMap.servo.get("fingers");

        // Extra Motor Steps
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);    //Reverse
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);     //Reverse

        //Mapping Distance Sensors
        //leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        //rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");

        //Mapping Color Sensors
        lfColorSensor = hardwareMap.get(NormalizedColorSensor.class, "lfColorSensor");
        if (lfColorSensor instanceof SwitchableLight) {
            ((SwitchableLight) lfColorSensor).enableLight(true);
        }
        rfColorSensor = hardwareMap.get(NormalizedColorSensor.class, "rfColorSensor");
        if (rfColorSensor instanceof SwitchableLight) {
            ((SwitchableLight) rfColorSensor).enableLight(true);
        }

        //IMU Mapping and Set-Up
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Initialize Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //OpenCv Set-Up
        pipeline = new OpenCVTest.SamplePipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //Reset Encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Initialization Complete
        telemetry.addData("Stat", "Start Program");
        telemetry.update();

        //Waiting for start via Player
        waitForStart();
    }

}
