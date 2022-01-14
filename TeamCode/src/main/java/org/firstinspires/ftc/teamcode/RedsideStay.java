package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Locale;

@Autonomous(name = "Redside")
public class RedsideStay extends LinearOpMode {
    // Declaring Motors & Servos
    private DcMotorEx leftFront;        //port 0
    private DcMotorEx rightFront;       //port 1
    private DcMotorEx leftBack;         //port 1
    private DcMotorEx rightBack;        //port 3
    private DcMotorEx spinner;
    private DcMotorEx arm;
    private Servo wrist;        //port 0
    private Servo fingers;         //port 0

    //Webcam Back
    private WebcamName Webcam2;

    //Declaring Distance Sensor Variables
    private DistanceSensor leftDistance;
    private DistanceSensor rightDistance;

    //Declaring Color Sensor Variables
    NormalizedColorSensor lfColorSensor;
    NormalizedColorSensor rfColorSensor;
    NormalizedRGBA colors;
    NormalizedRGBA colors2;
    boolean foundRed = false;
    boolean foundWhite = false;

    //Declaring IMU Variables
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double cumulativeHeading;
    boolean turned = false;

    //Declaring Camera Variables
    OpenCvCamera webcam;
    OpenCVTest.SamplePipeline pipeline;

    private static final String VUFORIA_KEY = "ARxaOAX/////AAABmR91q9ci+kNYqGb/NElhuhBQa5klidYZ5jKk5hFYJ6qAQOtCGKSEZXn1qYawipXKEEpJh+vP3GNnOUvabO2blz4vkymDnu8LUocLc6/rMpQdLwBt80JVdgWWkd/4j1DmwDdRRP4f/jP78furjgexjT7HgmC37xLP+msr78zAeWwkrsT2X1yjnL6nyiGcRKlBw6+EcUIcZYiiuXwbILds8rl4Fu7AuecLaygDft6XIUFg/qQm51UF45l5pYT8AoNTUhP9GTksKkmHgde7iGlo3CfIYu9QanjPHreT/+JZLJWG22jWC7Nnzch/1HC6s3s2jzkrFV6sRVA4lL9COLIonjRBYPhbxCF06c5fUMy9sj/e";
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vuforia    = null;
    private VuforiaTrackables targets   = null;
    private WebcamName webcamName       = null;

    private boolean targetVisible       = false;

    //int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    //VuforiaLocalizer.Parameters parameters2 = new VuforiaLocalizer.Parameters(cameraMonitorViewId2);


    final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
    final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

    //OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
          //  .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
          //  .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));


    @Override
    public void runOpMode() {
        initialize();

        while (opModeIsActive()) {
            encoders("off");

            if (pipeline.getType1().toString().equals("REDSQUARE") || pipeline.getType2().toString().equals("REDSQUARE") || pipeline.getType3().toString().equals("REDSQUARE")) {
                //On Blue Side

                if (true) {

                    //Distance Sensor on Right < x
                    //On Carousel Side

                    if (pipeline.getType1().toString().equals("DUCK")) {
                        //Duck in Field 1
                        telemetry.addData("Duck: ", "Field 1");
                        telemetry.addData("Robot: ", "Blue Carousel");
                        duckyData();

                        //fingers face forward, wrist opens, arm on bottom level
                        fingers.setPosition(0.1);
                        wrist.setPosition(0.25);
                        arm(21);

                        //start moving
                        moveInches(36,16);
                        turn(0.35,-80,"right",0.35);
                        //moveInches(4,16);

                        //drop off block
                        sleep(500);
                        fingers.setPosition(0.4);

                        //to wall
                        moveInches(-26,16);

                        //to ducky
                        turn(0.35,-20,"left",0.5);
                        moveInches(-30,16);

                        //deliver ducky
                        spinner(47.1238898038,10);

                        //fold in arm
                        fingers.setPosition(0.1);
                        wrist.setPosition(0.9);


                        //nap time
                        sleep(30000);


                    } else if (pipeline.getType2().toString().equals("DUCK")) {
                        //Duck in Field 2
                        telemetry.addData("Duck: ", "Field 2");
                        telemetry.addData("Robot: ", "Blue Carousel");
                        duckyData();

                        //fingers face forward, wrist opens, arm on bottom level
                        fingers.setPosition(0.1);
                        wrist.setPosition(0.3);
                        arm(26.5);

                        //start moving
                        moveInches(10,16);
                        turn(0.35,-30,"right",0.5);
                        moveInches(18,16);

                        //drop off block
                        sleep(500);
                        fingers.setPosition(0.3);

                        //to wall
                        moveInches(-18,16);

                        //to ducky
                        turn(0.35,-55,"right",0.5);
                        moveInches(-26.5,16);

                        //deliver ducky
                        spinner(47.1238898038,10);


                        //fold in arm
                        fingers.setPosition(0.1);
                        wrist.setPosition(0.9);


                        //nap time
                        sleep(30000);

                    } else if (pipeline.getType3().toString().equals("DUCK")) {
                        //Duck in Field 3
                        telemetry.addData("Duck: ", "Field 3");
                        telemetry.addData("Robot: ", "Blue Carousel");
                        duckyData();

                        //fingers face forward, wrist opens, arm on bottom level
                        fingers.setPosition(0.1);
                        wrist.setPosition(0.3);
                        arm(29.5);

                        //start moving
                        moveInches(10,16);
                        turn(0.35,-30,"right",0.5);
                        moveInches(18,16);

                        //drop off block
                        sleep(500);
                        fingers.setPosition(0.3);

                        //to wall
                        moveInches(-16,16);

                        //to ducky
                        turn(0.35,-55,"right",0.5);
                        moveInches(-26,16);
                        moveInches(-2,8);

                        //deliver ducky
                        spinner(47.1238898038,10);

                        //fold in arm
                        fingers.setPosition(0.1);
                        wrist.setPosition(0.9);


                        //nap time
                        sleep(30000);
                    }

                } else if (false) {
                    //Distance Sensor on Right > x
                    //On Storage Side

                    if (pipeline.getType1().toString().equals("DUCK")) {
                        //Duck in Field 1
                    } else if (pipeline.getType2().toString().equals("DUCK")) {
                        //Duck in Field 2
                    } else if (pipeline.getType3().toString().equals("DUCK")) {
                        //Duck in Field 3
                    }
                }

            } else if (pipeline.getType1().toString().equals("BLUESQUARE") || pipeline.getType2().toString().equals("BLUESQUARE") || pipeline.getType3().toString().equals("BLUESQUARE")) {
                //On Red Side
                if (true) {
                    //Distance Sensor on Left < x
                    //On Carousel Side

                    if (pipeline.getType1().toString().equals("DUCK")) {
                        //Duck in Field 1
                        telemetry.addData("Duck: ", "Field 1");
                        telemetry.addData("Robot: ", "Red Carousel");
                        duckyData();

                        fingers.setPosition(0.1);
                        wrist.setPosition(0.1);
                        arm(3.5);
                        moveInches(10,16);
                        turn(-0.35,-45,"right",0.33);
                        moveInches(22,16);
                        fingers.setPosition(0.3);

                    } else if (pipeline.getType2().toString().equals("DUCK")) {
                        //Duck in Field 2
                        telemetry.addData("Duck: ", "Field 2");
                        telemetry.addData("Robot: ", "Red Carousel");
                        duckyData();

                        fingers.setPosition(0.1);
                        wrist.setPosition(0.1);
                        arm(9);
                        //moveInches(10,16);
                        turn(-0.35,-45,"right",0.33);
                        moveInches(28,16);
                        fingers.setPosition(0.3);

                    } else if (pipeline.getType3().toString().equals("DUCK")) {
                        //Duck in Field 3
                        telemetry.addData("Duck: ", "Field 3");
                        telemetry.addData("Robot: ", "Red Carousel");
                        duckyData();

                        fingers.setPosition(0.1);
                        wrist.setPosition(0.1);
                        arm(3.5);
                        moveInches(18,16);
                        turn(-0.35,-80,"right",0.33);
                        moveInches(15,16);
                        wrist.setPosition(0.6);
                        sleep(500);
                        fingers.setPosition(0.3);
                    }

                } else if (false) {
                    //Distance Sensor on Left > x
                    //On Storage Side

                    if (pipeline.getType1().toString().equals("DUCK")) {
                        //Duck in Field 1
                    } else if (pipeline.getType2().toString().equals("DUCK")) {
                        //Duck in Field 2
                    } else if (pipeline.getType3().toString().equals("DUCK")) {
                        //Duck in Field 3
                    }
                }
            }
        }
    }


    public void duckyData() {
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Field 1", pipeline.getType1());
        telemetry.addData("AverageY", pipeline.getAverageY());
        telemetry.addData("AverageCr", pipeline.getAverageCr());
        telemetry.addData("AverageCb", pipeline.getAverageCb());
        telemetry.addLine();
        telemetry.addData("Field 2", pipeline.getType2());
        telemetry.addData("AverageY", pipeline.getAverageY2());
        telemetry.addData("AverageCr", pipeline.getAverageCr2());
        telemetry.addData("AverageCb", pipeline.getAverageCb2());
        telemetry.addLine();
        telemetry.addData("Field 3", pipeline.getType3());
        telemetry.addData("AverageY", pipeline.getAverageY3());
        telemetry.addData("AverageCr", pipeline.getAverageCr3());
        telemetry.addData("AverageCb", pipeline.getAverageCb3());

        telemetry.update();
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    //Method to Completely Stop ALL Robot Movement Excluding Servos
    private void forceStop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        spinner.setPower(0);
        arm.setPower(0);
    }

    //Method to Move Robot @ Designated Speed & Duration
    public void move(double speed, long dur) {
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
        sleep(dur);
    }

    double getBlueRedRatio(double ratio){
        return ratio;
    }
    double getBlueRedRatio2(double ratio2){
        return ratio2;
    }

    //Method to Find & Move to a White, Red, or Blue Line
    void senseLine(String color, double speed, String side) {
        final float[] hsvValues = new float[3];
        final float[] hsvValues2 = new float[3];
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initialHeading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        foundRed = false;
        foundWhite = false;
        int countRed = 0;
        int countWhite = 0;
        boolean shouldBreak = false;

        //If the "foundRed" Boolean is False, Run Loop
        while ((!shouldBreak) && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //Needed (updating) Variables
            NormalizedRGBA colors = lfColorSensor.getNormalizedColors();
            NormalizedRGBA colors2 = rfColorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            Color.colorToHSV(colors2.toColor(), hsvValues2);

            double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)) - initialHeading;
            double P = Math.abs(0.025 * heading);

            //If-Else-If Statement to Drive Forward or Backwards in a Straight Line
                if (heading < -0.1 && heading > -90) {
                    leftFront.setPower(speed - P);
                    leftBack.setPower(speed - P);
                    rightFront.setPower(speed + P);
                    rightBack.setPower(speed + P);
                } else if (heading > 0.1 && heading < 90) {
                    leftFront.setPower(speed + P);
                    leftBack.setPower(speed + P);
                    rightFront.setPower(speed - P);
                    rightBack.setPower(speed - P);
                } else {
                    leftFront.setPower(speed);
                    leftBack.setPower(speed);
                    rightFront.setPower(speed);
                    rightBack.setPower(speed);
                }

            double BlueRedRatio = colors.blue / colors.red;
            double BlueRedRatio2 = colors2.blue / colors2.red;
            if (side.equals("left")) {
                if ((BlueRedRatio < 0.83 || BlueRedRatio > 2.032)) {
                    telemetry.addLine("lfColorSensor says:");
                    if (BlueRedRatio <= 0.83 && (color.equals("red") || color.equals("Red"))) {
                        telemetry.addLine("Red Line Has Been Found! :)");
                        shouldBreak = true;
                    } else if ((BlueRedRatio >= 2.032) && (color.equals("blue") || color.equals("Blue"))) {
                        telemetry.addLine("Blue Line Has Been Found! :)");
                        shouldBreak = true;
                    }
                } else if ((colors.green * 1000 >= 9.041) && (color.equals("white") || color.equals("White"))) {
                    telemetry.addLine("lfColorSensor says:");
                    telemetry.addLine("White Line Has Been Found! :)");
                    shouldBreak = true;
                } else{
                    telemetry.addLine("lfColorSensor says:");
                    telemetry.addLine("Just Gray Tile :(");
                }
                telemetry.addLine();
            } else if (side.equals("right")) {
                if (BlueRedRatio2 < 0.83 || BlueRedRatio2 > 2.032) {
                    telemetry.addLine("rfColorSensor says:");
                    if (BlueRedRatio2 <= 0.83 && (color.equals("red") || color.equals("red"))) {
                        telemetry.addLine("Red Line Has Been Found! :)");
                        shouldBreak = true;
                    } else if (BlueRedRatio2 >= 2.032 && (color.equals("blue") || color.equals("Blue"))) {
                        telemetry.addLine("Blue Line Has Been Found! :)");
                        shouldBreak = true;
                    }
                } else if (colors2.green * 1000 >= 9.041 && (color.equals("white") || color.equals("White"))) {
                    telemetry.addLine("rfColorSensor says:");
                    telemetry.addLine("White Line Has Been Found! :)");
                    shouldBreak = true;
                } else{
                    telemetry.addLine("rfColorSensor says:");
                    telemetry.addLine("Just Gray Tile :(");
                }
            }


            telemetry.addLine();
            telemetry.addLine();

            telemetry.addLine("lfColorSensor: ");
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red * 1000)
                    .addData("Green", "%.3f", colors.green * 1000)
                    .addData("Blue", "%.3f", colors.blue * 1000);
            telemetry.addData("Blue/Red Ratio: ", Math.floor(getBlueRedRatio(BlueRedRatio)*1000)/1000);

            telemetry.addLine();

            telemetry.addLine("rfColorSensor: ");
            telemetry.addLine()
                    .addData("Red", "%.3f", colors2.red * 1000)
                    .addData("Green", "%.3f", colors2.green * 1000)
                    .addData("Blue", "%.3f", colors2.blue * 1000);
            telemetry.addData("Blue/Red Ratio: ", Math.floor(getBlueRedRatio2(BlueRedRatio2) * 1000)/1000);

            telemetry.addLine();

            //Telemetry Info for Diagnostics
            telemetry.addLine()
                    .addData("Heading Output", "%.3f", heading)
                    .addData("Loop Count", countRed);

            telemetry.update();

        }
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
    }


    //Method to Turn Robot Using IMU
    void turn(double speed, double angleMeasure, String direction, double tolerance) {
        double startHeading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        boolean turnLeft;
        boolean turnRight;

        if (direction.equals("left")||direction.equals("Left")) {
            turnLeft = true;
            turnRight = false;
        } else {
            turnLeft = false;
            turnRight = true;
        }

        turned = false;

        while (opModeIsActive() && !turned) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            startHeading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("approaching", angleMeasure);
            telemetry.addData("heading", startHeading);
            telemetry.update();
            if (turnLeft) {
                //turn left
                if ((angleMeasure - startHeading) >= 0) {
                    if ((startHeading <= (angleMeasure + tolerance)) && (startHeading >= (angleMeasure - tolerance))) {
                        forceStop();
                        turned = true;
                    } else if (startHeading >= (0.9 * angleMeasure) && startHeading < (angleMeasure - tolerance)) {
                        rightFront.setPower(0.5 * speed);
                        leftFront.setPower(0.5 * -speed);
                        rightBack.setPower(0.5 * speed);
                        leftBack.setPower(0.5 * -speed);
                    } else {
                        rightFront.setPower(speed);
                        leftFront.setPower(-speed);
                        rightBack.setPower(speed);
                        leftBack.setPower(-speed);
                    }
                } else {
                    if ((startHeading <= (angleMeasure + tolerance)) && (startHeading >= (angleMeasure - tolerance))) {
                        forceStop();
                        turned = true;
                    } else if (startHeading >= (0.9 * angleMeasure) && startHeading > (angleMeasure - tolerance)) {
                        rightFront.setPower(0.5 * speed);
                        leftFront.setPower(0.5 * -speed);
                        rightBack.setPower(0.5 * speed);
                        leftBack.setPower(0.5 * -speed);
                    } else {
                        rightFront.setPower(speed);
                        leftFront.setPower(-speed);
                        rightBack.setPower(speed);
                        leftBack.setPower(-speed);
                    }
                }
            } else if (turnRight) {
                //turn right
                if ((angleMeasure - startHeading) >= 0) {
                    if ((startHeading <= (angleMeasure + tolerance)) && (startHeading >= (angleMeasure - tolerance))) {
                        forceStop();
                        turned = true;
                    } else if (startHeading >= (0.9 * angleMeasure) && startHeading < (angleMeasure - tolerance)) {
                        rightFront.setPower(0.5 * -speed);
                        leftFront.setPower(0.5 * speed);
                        rightBack.setPower(0.5 * -speed);
                        leftBack.setPower(0.5 * speed);
                    } else {
                        rightFront.setPower(-speed);
                        leftFront.setPower(speed);
                        rightBack.setPower(-speed);
                        leftBack.setPower(speed);
                    }
                } else {
                    if (startHeading <= angleMeasure + tolerance && startHeading >= angleMeasure - tolerance) {
                        forceStop();
                        turned = true;
                    } else if (startHeading >= (0.9 * angleMeasure) && startHeading < (angleMeasure - tolerance)) {
                        rightFront.setPower(0.5 * -speed);
                        leftFront.setPower(0.5 * speed);
                        rightBack.setPower(0.5 * -speed);
                        leftBack.setPower(0.5 * speed);
                    } else {
                        rightFront.setPower(-speed);
                        leftFront.setPower(speed);
                        rightBack.setPower(-speed);
                        leftBack.setPower(speed);
                    }
                }
            }
        }
    }

    //Determine Whether To Run With Encoder
    public void encoders(String status) {
        if (status.equals("on")) {
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (status.equals("off")) {
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    void spinner (double distance, double velocity) {
        encoders("on");
        double calcRotation = -distance * 122.073;
        int setRotation = (int) Math.round(calcRotation);

        double calcVel = velocity * 122.073;
        int setVel =(int) Math.round(calcVel);

        spinner.setTargetPosition(setRotation);
        spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinner.setVelocity(setVel);

        while (opModeIsActive() && spinner.isBusy()) {
            telemetry.addData("position", spinner.getCurrentPosition());
            telemetry.update();
        }
        spinner.setVelocity(0);
        encoders("off");
    }

    void arm (double distance) {
        encoders("on");
        double calcUp = -66.9174 + (75.7517 * distance) + (17.8479 * distance * distance) + (-1.75956 * distance * distance * distance) + (0.0462714 * distance * distance * distance * distance);// (100 * (distance + 0.76442)) / 1.00529;
        int setUp = (int) Math.round(calcUp);

        double calcInc = (1000) ;
        int setInc =(int) Math.round(calcInc);
        if (distance < 0){
            setInc = (int) -Math.round(calcInc);
        }
        arm.setTargetPosition(setUp);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(setInc);

        while (opModeIsActive() && arm.isBusy()) {
            telemetry.addData("position", arm.getCurrentPosition());
            telemetry.update();
        }
        arm.setVelocity(0);
        encoders("off");
    }

    //Method to Move Robot Using Encoders
    void moveInches(double distance, double velocity) {

        encoders("on");

        double calcPosition = distance * 100.531;
        int setPosition = (int) Math.round(calcPosition);

        double calcVelocity = velocity * 100.531;
        int setVelocity = (int) Math.round(calcVelocity);

        leftFront.setTargetPosition(setPosition);
        rightFront.setTargetPosition(setPosition);
        leftBack.setTargetPosition(setPosition);
        rightBack.setTargetPosition(setPosition);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setVelocity(setVelocity);
        rightFront.setVelocity(setVelocity);
        leftBack.setVelocity(setVelocity);
        rightBack.setVelocity(setVelocity);


        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() ) {

            telemetry.addData("position", leftFront.getCurrentPosition());
            telemetry.addData("position", rightFront.getCurrentPosition());
            telemetry.addData("position", leftBack.getCurrentPosition());
            telemetry.addData("position", rightBack.getCurrentPosition());
            telemetry.addData("is at target", !leftFront.isBusy());
            telemetry.update();
        }

        leftFront.setVelocity(0);
        rightFront.setVelocity(0);
        leftBack.setVelocity(0);
        rightBack.setVelocity(0);

        encoders("off");
    }

    public static class SamplePipeline extends OpenCvPipeline {
        private static final Scalar BLUE = new Scalar(0, 0, 255);

        Point topLeft = new Point(0, 160);
        Point bottomRight = new Point(50, 210);
        Point topLeft2 = new Point(130, 150);
        Point bottomRight2 = new Point(180, 200);
        Point topLeft3 = new Point(265, 140);
        Point bottomRight3 = new Point(315, 190);

        Mat region1_Y;
        Mat region1_Cr;
        Mat region1_Cb;
        Mat region2_Y;
        Mat region2_Cr;
        Mat region2_Cb;
        Mat region3_Y;
        Mat region3_Cr;
        Mat region3_Cb;
        Mat YCrCb = new Mat();
        Mat Y = new Mat();
        Mat Cr = new Mat();
        Mat Cb = new Mat();

        private volatile int averageY;
        private volatile int averageCr;
        private volatile int averageCb;
        private volatile int averageY2;
        private volatile int averageCr2;
        private volatile int averageCb2;
        private volatile int averageY3;
        private volatile int averageCr3;
        private volatile int averageCb3;
        private volatile OpenCVTest.SamplePipeline.TYPE type1 = OpenCVTest.SamplePipeline.TYPE.NULL;
        private volatile OpenCVTest.SamplePipeline.TYPE type2 = OpenCVTest.SamplePipeline.TYPE.NULL;
        private volatile OpenCVTest.SamplePipeline.TYPE type3 = OpenCVTest.SamplePipeline.TYPE.NULL;

        private void inputToY(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Y, 0);
        }

        private void inputToCr(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 1);
        }

        private void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }


        @Override
        public void init(Mat input) {
            inputToY(input);
            inputToCr(input);
            inputToCb(input);

            region1_Y = Y.submat(new Rect(topLeft, bottomRight));
            region1_Cr = Cr.submat(new Rect(topLeft, bottomRight));
            region1_Cb = Cb.submat(new Rect(topLeft, bottomRight));

            region2_Y = Y.submat(new Rect(topLeft2, bottomRight2));
            region2_Cr = Cr.submat(new Rect(topLeft2, bottomRight2));
            region2_Cb = Cb.submat(new Rect(topLeft2, bottomRight2));

            region3_Y = Y.submat(new Rect(topLeft3, bottomRight3));
            region3_Cr = Cr.submat(new Rect(topLeft3, bottomRight3));
            region3_Cb = Cb.submat(new Rect(topLeft3, bottomRight3));
        }

        @Override
        public Mat processFrame(Mat input) {

            inputToY(input);
            inputToCr(input);
            inputToCb(input);

            averageY = (int) Core.mean(region1_Y).val[0];
            averageCr = (int) Core.mean(region1_Cr).val[0];
            averageCb = (int) Core.mean(region1_Cb).val[0];

            averageY2 = (int) Core.mean(region2_Y).val[0];
            averageCr2 = (int) Core.mean(region2_Cr).val[0];
            averageCb2 = (int) Core.mean(region2_Cb).val[0];

            averageY3 = (int) Core.mean(region3_Y).val[0];
            averageCr3 = (int) Core.mean(region3_Cr).val[0];
            averageCb3 = (int) Core.mean(region3_Cb).val[0];

            Imgproc.rectangle(input, topLeft, bottomRight, BLUE, 2);
            Imgproc.rectangle(input, topLeft2, bottomRight2, BLUE, 2);
            Imgproc.rectangle(input, topLeft3, bottomRight3, BLUE, 2);


            if (((double)getAverageCb())/(double)getAverageCr() < 0.9547) {
                type1 = OpenCVTest.SamplePipeline.TYPE.REDSQUARE;
            } else if (((double)getAverageCb())/(double)getAverageCr() < 1.1234) {
                type1 = OpenCVTest.SamplePipeline.TYPE.DUCK;
            } else if (((double)getAverageCb())/(double)getAverageCr() < 150) {
                type1 = OpenCVTest.SamplePipeline.TYPE.BLUESQUARE;
            } else {
                type1 = OpenCVTest.SamplePipeline.TYPE.NULL;
            }

            if (((double)getAverageCb2())/(double)getAverageCr2() < 0.9547) {
                type2 = OpenCVTest.SamplePipeline.TYPE.REDSQUARE;
            } else if (((double)getAverageCb2())/(double)getAverageCr2() < 1.1234) {
                type2 = OpenCVTest.SamplePipeline.TYPE.DUCK;
            } else if ((((double)getAverageCb2())/(double)getAverageCr2()) < 150) {
                type2 = OpenCVTest.SamplePipeline.TYPE.BLUESQUARE;
            } else {
                type2 = OpenCVTest.SamplePipeline.TYPE.NULL;
            }

            if ((((double)getAverageCb3())/(double)getAverageCr3()) < 0.9547) {
                type3 = OpenCVTest.SamplePipeline.TYPE.REDSQUARE;
            } else if ((((double)getAverageCb3())/(double)getAverageCr3()) <1.1234) {
                type3 = OpenCVTest.SamplePipeline.TYPE.DUCK;
            } else if ((((double)getAverageCb3())/(double)getAverageCr3()) < 150) {
                type3 = OpenCVTest.SamplePipeline.TYPE.BLUESQUARE;
            } else {
                type3 = OpenCVTest.SamplePipeline.TYPE.NULL;
            }


            return input;
        }


        public OpenCVTest.SamplePipeline.TYPE getType1() {
            return type1;
        }

        public OpenCVTest.SamplePipeline.TYPE getType2() {
            return type2;
        }

        public OpenCVTest.SamplePipeline.TYPE getType3() {
            return type3;
        }

        public int getAverageY() {
            return averageY;
        }

        public int getAverageCr() {
            return averageCr;
        }

        public int getAverageCb() {
            return averageCb;
        }

        public int getAverageY2() {
            return averageY2;
        }

        public int getAverageCr2() {
            return averageCr2;
        }

        public int getAverageCb2() {
            return averageCb2;
        }

        public int getAverageY3() {
            return averageY3;
        }

        public int getAverageCr3() {
            return averageCr3;
        }

        public int getAverageCb3() {
            return averageCb3;
        }


        public enum TYPE {
            BALL, BLUESQUARE, CUBE, DUCK, NULL, REDSQUARE
        }
    }

    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public void initialize() {
        telemetry.addData("Stat", "Initializing...");
        telemetry.update();

        //Mapping Motors
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 2");

        //Mapping Servos
        wrist = hardwareMap.servo.get("wrist");
        fingers = hardwareMap.servo.get("fingers");

        // Extra Motor Steps
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);    //Reverse
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);     //Reverse

        //Mapping Distance Sensors
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");

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
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


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
// Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 2");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

       // parameters2.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
       // parameters2.cameraName = webcamName;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
       // parameters2.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
       // vuforia = ClassFactory.getInstance().createVuforia(parameters2);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        //targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");


        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        //identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        //identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        //identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        //identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        /*
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
         * with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */

        /**  Let all the trackable listeners know where the camera is.  */


        /*
         * WARNING:
         * In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
         * This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
         * CONSEQUENTLY do not put any driving commands in this loop.
         * To restore the normal opmode structure, just un-comment the following line:
         */


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

