package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@TeleOp
public class OpenCVTest extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Field 1", pipeline.getType1());
            telemetry.addData("AverageY", pipeline.getAverageY());
            telemetry.addData("AverageCr", pipeline.getAverageCr());
            telemetry.addData("AverageCb", pipeline.getAverageCb());

            telemetry.addData("Field 2", pipeline.getType2());
            telemetry.addData("AverageY", pipeline.getAverageY2());
            telemetry.addData("AverageCr", pipeline.getAverageCr2());
            telemetry.addData("AverageCb", pipeline.getAverageCb2());

            telemetry.addData("Field 3", pipeline.getType3());
            telemetry.addData("AverageY", pipeline.getAverageY3());
            telemetry.addData("AverageCr", pipeline.getAverageCr3());
            telemetry.addData("AverageCb", pipeline.getAverageCb3());

            telemetry.update();
            sleep(100);
        }
    }

    public static class SamplePipeline extends OpenCvPipeline {
        private static final Scalar BLUE = new Scalar(0, 0, 255);

        private static final int THRESHOLD = 107;

        Point topLeft = new Point(0, 150);
        Point bottomRight = new Point(50, 200);
        Point topLeft2 = new Point(125, 150);
        Point bottomRight2 = new Point(175, 200);
        Point topLeft3 = new Point(250, 150);
        Point bottomRight3 = new Point(300, 200);

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
        private volatile TYPE type1 = TYPE.NULL;
        private volatile TYPE type2 = TYPE.NULL;
        private volatile TYPE type3 = TYPE.NULL;

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

            if (averageY > 30 && averageY < 130 && averageCr < 200 && averageCr > 100 && averageCb < 120) {
                type1 = TYPE.DUCK;
            } else if (averageY > 75 && averageY < 125 && averageCr < 150 && averageCr > 100 && averageCb > 100 && averageCb < 150) {
                type1 = TYPE.BLUESQUARE;
            } else if (averageY < 130 && averageY > 30 && averageCr > 190 && averageCb < 140 && averageCb> 40) {
                type1 = TYPE.REDSQUARE;
            } else {
                type1 = TYPE.NULL;
            }

            if (averageY2 > 30 && averageY2 < 130 && averageCr2 < 200 && averageCr2 > 100 && averageCb2 < 120) {
                type2 = TYPE.DUCK;
            } else if (averageY2 > 75 && averageY2 < 125 && averageCr2 < 150 && averageCr2 > 100 && averageCb2 > 100 && averageCb2 < 150) {
                type2 = TYPE.BLUESQUARE;
            } else if (averageY2 < 130 && averageY2 > 30 && averageCr2 > 190 && averageCb2 < 140 && averageCb2> 40) {
                type2 = TYPE.REDSQUARE;
            } else {
                type2 = TYPE.NULL;
            }


            if (averageY3 > 30 && averageY3 < 130 && averageCr3 < 200 && averageCr3 > 100 && averageCb3 < 120) {
                type3 = TYPE.DUCK;
            } else if (averageY3 > 75 && averageY3 < 125 && averageCr3 < 150 && averageCr3 > 100 && averageCb3 > 100 && averageCb3 < 150) {
                type3 = TYPE.BLUESQUARE;
            } else if (averageY3 < 130 && averageY3 > 30 && averageCr3 > 190 && averageCb3 < 140 && averageCb3> 40) {
                type3 = TYPE.REDSQUARE;
            } else {
                type3 = TYPE.NULL;
            }

            return input;
        }


        public TYPE getType1() {
            return type1;
        }
        public TYPE getType2() {
            return type2;
        }
        public TYPE getType3() {
            return type2;
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
            BALL, CUBE, DUCK, REDSQUARE, BLUESQUARE, NULL
        }
    }
}