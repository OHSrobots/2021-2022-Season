package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

@Autonomous(name = "FinalAuto", group = "comp")
public class FinalAuto extends LinearOpMode {

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
    NormalizedRGBA colors2;
    boolean foundRed = false;
    boolean foundWhite = false;

    //Declaring IMU Variables
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double initialHeading;
    boolean turned = false;

    //Declaring Camera Variables
    OpenCvCamera webcam;
    OpenCVTest.SamplePipeline pipeline;

    @Override
    public void runOpMode() {
        initialize();

        while (opModeIsActive()) {
            encoders("off");

            if (pipeline.getType1().toString().equals("BLUESQUARE") || pipeline.getType2().toString().equals("BLUESQUARE") || pipeline.getType3().toString().equals("BLUESQUARE")) {
                //On Blue Side

                if (true/*rightDistance.getDistance(DistanceUnit.CM) < 80.00*/) {
                    //Distance Sensor on Right < x
                    //On Carousel Side

                    if (pipeline.getType1().toString().equals("DUCK")) {
                        //Duck in Field 1
                        telemetry.addData("Duck: ", "Field 1");
                        telemetry.addData("Robot: ", "Blue Carousel");
                        duckyData();

                        //Color Sensor
                        senseLine("blue", 0.25);
                        turn(0.4, 0);
                        turn(0.4, 180);
                        turn(0.4, 0);
                        turn(0.4, -180);

                    } else if (pipeline.getType2().toString().equals("DUCK")) {
                        //Duck in Field 2
                        telemetry.addData("Duck: ", "Field 2");
                        telemetry.addData("Robot: ", "Blue Carousel");
                        duckyData();

                    } else if (pipeline.getType3().toString().equals("DUCK")) {
                        //Duck in Field 3
                        telemetry.addData("Duck: ", "Field 3");
                        telemetry.addData("Robot: ", "Blue Carousel");
                        duckyData();
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

            } else if (pipeline.getType1().toString().equals("REDSQUARE") || pipeline.getType2().toString().equals("REDSQUARE") || pipeline.getType3().toString().equals("REDSQUARE")) {
                //On Red Side
                if (true) {
                    //Distance Sensor on Left < x
                    //On Carousel Side

                    if (pipeline.getType1().toString().equals("DUCK")) {
                        //Duck in Field 1
                        telemetry.addData("Duck: ", "Field 1");
                        telemetry.addData("Robot: ", "Red Carousel");
                        duckyData();

                    } else if (pipeline.getType2().toString().equals("DUCK")) {
                        //Duck in Field 2
                        telemetry.addData("Duck: ", "Field 2");
                        telemetry.addData("Robot: ", "Red Carousel");
                        duckyData();

                    } else if (pipeline.getType3().toString().equals("DUCK")) {
                        //Duck in Field 3
                        telemetry.addData("Duck: ", "Field 3");
                        telemetry.addData("Robot: ", "Red Carousel");
                        duckyData();
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
            sleep(3000);
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
    void senseLine(String color, double speed) {
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

            if (BlueRedRatio2 < 0.83 || BlueRedRatio2 > 2.032) {
                telemetry.addLine("rfColorSensor says:");
                if (BlueRedRatio2 <= 0.83 && (color.equals("red") || color.equals("red"))) {
                    telemetry.addLine("Red Line Has Been Found! :)");
                } else if (BlueRedRatio2 >= 2.032 && (color.equals("blue") || color.equals("Blue"))) {
                    telemetry.addLine("Blue Line Has Been Found! :)");
                }
            } else if (colors2.green * 1000 >= 9.041 && (color.equals("white") || color.equals("White"))) {
                telemetry.addLine("rfColorSensor says:");
                telemetry.addLine("White Line Has Been Found! :)");
            } else{
                telemetry.addLine("rfColorSensor says:");
                telemetry.addLine("Just Gray Tile :(");
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
    void turn(double speed, int angleMeasure) {
        turned = false;
        while (opModeIsActive() && !turned) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double startHeading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            telemetry.addData("heading", startHeading);
            telemetry.update();
            if (angleMeasure - startHeading >= 0) {
                //turn left
                if (startHeading <= angleMeasure + 1 && startHeading >= angleMeasure - 1) {
                    forceStop();
                    turned = true;
                } else if (startHeading >= (0.9 * angleMeasure) && startHeading < (angleMeasure - 1)) {
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

            } else if (angleMeasure - startHeading <= 0 ) {
                //turn right
                if (startHeading <= angleMeasure + 1 && startHeading >= angleMeasure - 1) {
                    forceStop();
                    turned = true;
                } else if (startHeading >= (0.9 * angleMeasure) && startHeading < (angleMeasure - 1)) {
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

    //Determine Whether To Run With Encoder
    public void encoders(String status) {
        if (status.equals("on")) {
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (status.equals("off")) {
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //Method to Move Robot Using Encoders
    void moveInches(double distance, double velocity) {

        encoders("on");

        double calcPosition = distance * (100 * 280 / (16.9646003294 * 4 * 8.8 * 1.0555555556));
        int setPosition = (int) Math.round(calcPosition);

        int setVelocity = (int) Math.round(velocity);

        leftFront.setTargetPosition(setPosition);
        rightFront.setTargetPosition(setPosition);
        leftBack.setTargetPosition(setPosition);
        rightBack.setTargetPosition(setPosition);

        leftFront.setVelocity(setVelocity);
        rightFront.setVelocity(setVelocity);
        leftBack.setVelocity(setVelocity);
        rightBack.setVelocity(setVelocity);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && leftFront.isBusy()) {
            telemetry.addData("position", leftFront.getCurrentPosition());
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

            if (averageCb < 120 && averageCr < 150) {
                type1 = OpenCVTest.SamplePipeline.TYPE.DUCK;
            } else if (averageCb >= 120) {
                type1 = OpenCVTest.SamplePipeline.TYPE.BLUESQUARE;
            } else if (averageCr >= 150) {
                type1 = OpenCVTest.SamplePipeline.TYPE.REDSQUARE;
            } else {
                type1 = OpenCVTest.SamplePipeline.TYPE.NULL;
            }

            if (averageCb2 < 120 && averageCr2 < 150) {
                type2 = OpenCVTest.SamplePipeline.TYPE.DUCK;
            } else if (averageCb2 >= 120) {
                type2 = OpenCVTest.SamplePipeline.TYPE.BLUESQUARE;
            } else if (averageCr2 >= 150) {
                type2 = OpenCVTest.SamplePipeline.TYPE.REDSQUARE;
            } else {
                type2 = OpenCVTest.SamplePipeline.TYPE.NULL;
            }


            if (averageCb3 < 120 && averageCr3 < 150) {
                type3 = OpenCVTest.SamplePipeline.TYPE.DUCK;
            } else if (averageCb3 >= 120) {
                type3 = OpenCVTest.SamplePipeline.TYPE.BLUESQUARE;
            } else if (averageCr3 >= 150) {
                type3 = OpenCVTest.SamplePipeline.TYPE.REDSQUARE;
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

