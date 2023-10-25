package org.firstinspires.ftc.teamcode.hardware.sensors.pipelines;//package org.firstinspires.ftc.teamcode.Hardware.Sensors;
//
//import org.firstinspires.ftc.teamcode.Utilities.Vector2D;
//import org.opencv.core.*;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.imgproc.Moments;
//import org.opencv.objdetect.Objdetect;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
//
//public class GoalPositionPipeline extends OpenCvPipeline {
//
//
//        double[] hsvArr = new double[3];
//
//        Mat mask = new Mat();
//
//
//        /*
//         * Points which actually define the sample region rectangles, derived from above values
//         *
//         * Example of how points A and B work to define a rectangle
//         *
//         *   ------------------------------------
//         *   | (0,0) Point A                    |
//         *   |                                  |
//         *   |                                  |
//         *   |                                  |
//         *   |                                  |
//         *   |                                  |
//         *   |                                  |
//         *   |                  Point B (70,50) |
//         *   ------------------------------------
//         *
//         */
//
//        float centerXFirst = -1;
//        float centerYFirst = -1;
//
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//
//        private volatile int height = -1;
//        private volatile double heightWidthRatio = 0;
//
//        Rect boundingRect = new Rect(-1,-1,-1,-1);
//        Rect boundingRect2 = new Rect(-1,-1,-1,-1);
//        Rect finalBoundingRect = new Rect(-1,-1,-1,-1);
//
//
//        void inputToCb(Mat input)
//        {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 2);
//        }
//
//        @Override
//        public void init(Mat firstFrame)
//        {
//
//            inputToCb(firstFrame);
//
//        }
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//
//            final Scalar lower = new Scalar(0, 150, 150);
//            final Scalar upper = new Scalar(8, 255, 230);
//
//
//            Mat hsv = new Mat();
//            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
//
//
//            Core.inRange(hsv, lower, upper, mask);
//
//
//            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((4) + 1, (2)+1));
//            Imgproc.erode(mask, mask, kernel);
//            Imgproc.dilate(mask, mask, kernel);
//
//
//
////            List<MatOfPoint> contours = new ArrayList<>();
////            Mat hierarchy = new Mat();
////            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
////            double maxArea = 0;
////            double secondMaxArea = 0;
////            MatOfPoint firstContour = new MatOfPoint();
////            MatOfPoint secondContour = new MatOfPoint();
////            for (MatOfPoint contour : contours) {
////                double area = Imgproc.contourArea(contour);
////                if (area > maxArea) {
////                    maxArea = area;
////                    firstContour = contour;
////                } else if (area > secondMaxArea) {
////                    secondMaxArea = area;
////                    secondContour = contour;
////                }
////            }
////
////            boundingRect2 = Imgproc.boundingRect(secondContour);
//
//            Moments mFirst = Imgproc.moments(mask, true);
//
//            centerXFirst = (float)(mFirst.m10 / mFirst.m00);
//            centerYFirst = (float)(mFirst.m01 / mFirst.m00);
//            if(Float.isNaN(centerXFirst)||Float.isNaN(centerYFirst)){
//                centerXFirst = -1;
//                centerYFirst = -1;
//            }
//
//            hsvArr = hsv.get(120, 160);
//            Imgproc.putText(input, Double.toString(hsvArr[0]) + ' ' + Double.toString(hsvArr[1]) + ' ' + Double.toString(hsvArr[2]), new Point(100, 100), 1, 1, new Scalar( 0, 255, 0));
//            Imgproc.circle(input, new Point(160, 120), 5, new Scalar(255, 0, 0));
//
//            Imgproc.circle(mask, new Point(centerXFirst, centerYFirst), 5, new Scalar(255, 0 ,0));
//
////            int lowestX = boundingRect.x<boundingRect2.x?boundingRect.x:boundingRect2.x;
////            int lowestY = boundingRect.y<boundingRect2.y?boundingRect.y:boundingRect2.y;
////            int highestX = boundingRect.x+boundingRect.width>boundingRect2.x+boundingRect2.width?boundingRect.x+boundingRect.width:boundingRect2.x+boundingRect2.width;
////            int highestY = boundingRect.y+boundingRect.height>boundingRect2.y+boundingRect2.height?boundingRect.y+boundingRect.height:boundingRect2.y+boundingRect2.height;
////            //new MatOfRect(boundingRect, boundingRect2)
//
////            finalBoundingRect = new Rect (lowestX, lowestY, highestX-lowestX, highestY-lowestY);
////
////
////            Imgproc.rectangle(input, boundingRect, new Scalar(60, 255, 255));
////            Imgproc.putText(input, Integer.toString(lowestX) + " " + Integer.toString(lowestY) + " " + Integer.toString(highestX) + " " + Integer.toString(highestY), new Point(100, 100), 1, 1, new Scalar( 255, 255, 255));
////            Imgproc.rectangle(input, boundingRect2, new Scalar(0, 255, 255));
////            Imgproc.rectangle(input, finalBoundingRect, new Scalar(50, 50, 50));
////
////            Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2RGB);
//
//
//            hsv.release();
//            //hierarchy.release();
//            //kernel.release();
//
//
//            return input;
//        }
//
//        /*
//         * Call this from the OpMode thread to obtain the latest analysis
//         */
//        public Vector2D getPos(){ return new Vector2D(centerXFirst, centerYFirst); }
//        public double[] hsvValues() {return hsvArr;}
//}
