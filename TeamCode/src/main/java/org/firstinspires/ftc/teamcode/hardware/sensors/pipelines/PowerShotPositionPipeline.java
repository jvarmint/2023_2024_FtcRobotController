package org.firstinspires.ftc.teamcode.hardware.sensors.pipelines;//package org.firstinspires.ftc.teamcode.Hardware.Sensors;
//
//import android.util.Pair;
//import org.firstinspires.ftc.teamcode.Utilities.Vector2D;
//import org.opencv.core.*;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.Collections;
//import java.util.Comparator;
//import java.util.List;
//
//
//public class PowerShotPositionPipeline extends OpenCvPipeline {
//
//    static class PairComparator implements Comparator<Pair<Double, MatOfPoint>>{
//
//        @Override
//        public int compare(Pair<Double, MatOfPoint> doubleMatOfPointPair, Pair<Double, MatOfPoint> t1) {
//            return doubleMatOfPointPair.first.compareTo(t1.first);
//        }
//    }
//
//    static class HeightComparator implements  Comparator<Rect>{
//
//        @Override
//        public int compare(Rect rect, Rect t1) {
//            if(t1.empty() || rect.empty()) return 0;
//            if(0.25*rect.width>Math.abs(rect.width-t1.width) || 0.25*t1.width>Math.abs(rect.width-t1.width)) {
//                return 0;
//            } else if (rect.width>t1.width){
//                return 1;//was 1
//            } else {
//                return -1;///was -1
//            }
//        }
//    }
//
//    static class LeftRightComparator implements  Comparator<Rect>{
//
//        @Override
//        public int compare(Rect rect, Rect t1) {
//            if(t1.empty() || rect.empty()) return 0;
//
//            if(rect.y>t1.y) return 1;
//
//             else return -1;
//
//        }
//    }
//        Mat mask = new Mat();
//
//        ArrayList<Rect> rectArray = new ArrayList<Rect>();
//
//        Vector2D retV = new Vector2D(-1, -1);
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
//
//        Rect boundingRect = new Rect(-1,-1,-1,-1);
//        Rect boundingRect2 = new Rect(-1,-1,-1,-1);
//        Rect boundingRect3 = new Rect(-1,-1,-1,-1);
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
//            try {
//                final Scalar lower = new Scalar(0, 110, 50);
//                final Scalar upper = new Scalar(10, 255, 240);
//
//                Mat nHSV = new Mat();
//
//                Imgproc.cvtColor(input, nHSV, Imgproc.COLOR_RGB2HSV);
//
//                Rect rectCrop = new Rect(0, 0, input.width()/2, input.height());
//                Mat hsv = new Mat(nHSV, rectCrop);
//
//                Core.inRange(hsv, lower, upper, mask);
//
//                Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2) + 1, (2) + 1));
//                Imgproc.erode(mask, mask, kernel);
//                Imgproc.dilate(mask, mask, kernel);
//
//
//                List<MatOfPoint> contours = new ArrayList<>();
//                Mat hierarchy = new Mat();
//                Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//                ArrayList<Pair<Double, MatOfPoint>> doubleArrayList = new ArrayList<Pair<Double, MatOfPoint>>();
//                for (MatOfPoint contour : contours) {
//                    if (Imgproc.boundingRect(contour).height > 100 ||
//                        Imgproc.boundingRect(contour).width > 150 ||
//                        Imgproc.boundingRect(contour).width < 90 ||
//                        Imgproc.boundingRect(contour).height < 5) continue;
//
//                    doubleArrayList.add(new Pair<Double, MatOfPoint>((double) Imgproc.boundingRect(contour).width, contour));
//                }
//
//                //Sort by real-world-height
//                if (!doubleArrayList.isEmpty()) {
//                    Collections.sort(doubleArrayList, new PairComparator());
//                    Collections.reverse(doubleArrayList);
//                }
//
//                // Create bounding rects
//                int i = 0;
//                rectArray.clear();
//                for (Pair<Double, MatOfPoint> p : doubleArrayList) {
//                    Rect b = Imgproc.boundingRect(p.second);
//                    if (b.empty()) continue;
//
//                    rectArray.add(b);
//                    Imgproc.rectangle(mask, Imgproc.boundingRect(p.second), new Scalar(255, 0, 0), 4);
//                    i++;
//                    //Limiting to 3 rects
////                  if (i > 0) break;
//                }
//
//
////            if(doubleArrayList.size()>2) {
////                Imgproc.putText(mask, Double.toString(Imgproc.boundingRect(doubleArrayList.get(0).second).area()) + " " + Double.toString(Imgproc.boundingRect(doubleArrayList.get(0).second).width), new Point(100, 100), 1, 3, new Scalar(255, 0, 0));
////            }
//
//
//
//
//                List<Rect> rArray;
////                if(!rectArray.isEmpty()) {
////                    try {
//                        Collections.sort(rectArray, new LeftRightComparator());
//                        Collections.reverse(rectArray);
////                        if(rectArray.size() > 2) {
////                            rArray = rectArray.subList(0, 2);
////                            Collections.sort(rArray, new HeightComparator());
////                            for (Rect r : rArray) {
////                                Imgproc.rectangle(mask, r, new Scalar(0, 255, 255), 4);
////                            }
////                        }
//
//
////
////                    } catch (Exception e) {
////                        rArray = null;
////                    }
////
////
////                }
//
//                hsv.release();
//                hierarchy.release();
//                nHSV.release();
//                kernel.release();
//                return mask;
//            } catch (Exception e){
//                return null;
//            }
//        }
//
//        /*
//         * Call this from the OpMode thread to obtain the latest analysis
//         */
//        public Vector2D getPowerCenter() {
//            if (!rectArray.isEmpty()) {
//                try {
//                    //sort left right
//                    Collections.sort(rectArray, new LeftRightComparator());
//                    Collections.reverse(rectArray);
//                    return new Vector2D(rectArray.get(0).width/*rectArray.get(0).x + rectArray.get(0).width / 2.0*/, rectArray.get(0).y + rectArray.get(0).height / 2.0);
//                    //                    List<Rect> rArray = rectArray.subList(0, 2);
////                    Collections.sort(rArray, new HeightComparator());
////                    return new Vector2D(rArray.get(0).x + rArray.get(0).width / 2.0, rArray.get(0).y + rArray.get(0).height / 2.0);
//
//                }catch(Exception e)
//                {
//                    return new Vector2D(-1, -1);
//                }
//            } else {
//                return new Vector2D(-1, -1);
//            }
//        }
//}
