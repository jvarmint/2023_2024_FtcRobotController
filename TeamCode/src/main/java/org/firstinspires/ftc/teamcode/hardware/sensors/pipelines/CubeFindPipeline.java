package org.firstinspires.ftc.teamcode.hardware.sensors.pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class CubeFindPipeline extends OpenCvPipeline {
    Mat grey = new Mat();
    Mat hsv = new Mat();
    Mat mask = new Mat();
    Mat hierarchy = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    volatile double cubeXValueCommand = 0;
    volatile boolean cubeXValueCommandGood = false;
    volatile boolean cubeClose = false;
    volatile boolean duckClose = false;
    final int WIDTH = 640;
    final int HEIGHT = 480;
    final Scalar lower = new Scalar(4, 50, 20);//150, 90
    final Scalar upper = new Scalar(36, 255, 255);// 230, 255


    static class HeightComparator implements Comparator<Point> {

//        @Override
//        public int compare(MatOfPoint t1, MatOfPoint t2) {
//            if(t1.empty() || t2.empty()) return 0;
//            if(t1.>Math.abs(rect.width-t1.width) || 0.25*t1.width>Math.abs(rect.width-t1.width)) {
//                return 0;
//            } else if (rect.width>t1.width){
//                return 1;//was 1
//            } else {
//                return -1;///was -1
//            }
//            return 0;
//        }

        @Override
        public int compare(Point p1, Point p2) {
            if(p1.y == p2.y) return 0;
            else if (p1.y < p2.y) return 1;
            else return -1;
        }
    }



    @Override
    public Mat processFrame(Mat input) {
//        final Scalar lower = new Scalar(0, 0, 20);//188.1, 148.92);
//        final Scalar upper = new Scalar(50, 255, 250);//203.7, 255);



        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);


        Core.inRange(hsv, lower, upper, mask);


//        grey.empty();
//        Core.add(grey, new Scalar(255, 0, 0), grey);
//        input.copyTo(grey, mask);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(33, 33));
        Imgproc.erode(mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);

        int trueTotal = 0;
        for(int i = 0; i<mask.width(); i++){
            if(mask.get(mask.height()-1, i)[0]>0.0) {
                trueTotal += 1;
            }

        }
        cubeClose = (double)(trueTotal)/(double)mask.width() > .65;
        boolean possibleDuck = (double)(trueTotal)/(double)mask.width() > .1;
//
        Mat cannyEdges = new Mat();
        Imgproc.Canny(mask, cannyEdges, 10, 100);

//        MatOfPoint totalContours = new MatOfPoint();
        List<MatOfPoint> tempContours = new ArrayList<>();
        Imgproc.findContours(cannyEdges, tempContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        contours = tempContours;
//        for(int i =0; i<contours.size(); i++) {
//            totalContours.push_back(contours.get(i));
//        }
//        List<Point> listOfPoint = totalContours.toList();
//        Collections.sort(listOfPoint, new HeightComparator());
        MatOfPoint totalContours = new MatOfPoint();
        for (int i = 0; i < contours.size(); i++) {
            totalContours.push_back(contours.get(i));
        }
        boolean duckFound = false;
        try {
            List<Point> listOfPoint = totalContours.toList();
            Collections.sort(listOfPoint, new HeightComparator());
//            double scalarY = 1.0-(listOfPoint.get(0).y/((double)HEIGHT));
            cubeXValueCommand =  0.1*(((listOfPoint.get(0).x+listOfPoint.get(1).x) / 2.0) / (WIDTH/2.0) - 1.0);
            cubeXValueCommandGood = true;

            if(possibleDuck){
                for(int i = listOfPoint.size()-1; i>=0; i--){
                    if(listOfPoint.get(i).y > HEIGHT/2.0){
                        duckFound = listOfPoint.get(i).y > HEIGHT*0.75;
                        break;
                    }
                }
            }

        } catch (Exception e){
            cubeXValueCommandGood = false;
        }
        duckClose = duckFound;


        for(int i =0; i<contours.size(); i++) {
            Imgproc.drawContours(input, contours, i, new Scalar(100, 50, 100), 5);
        }
//        Imgproc.putText(input, Double.toString(listOfPoint.get(0).x) + ":" + Double.toString(listOfPoint.get(0).y), new Point(100, 100), FONT_HERSHEY_SIMPLEX, 5, new Scalar(100, 0, 100), 4);
//        double maxArea = 0;
//        MatOfPoint largestContour = new MatOfPoint();
//        for (MatOfPoint contour : contours) {
//            double area = Imgproc.contourArea(contour);
//            if (area > maxArea) {
//                maxArea = area;
//                largestContour = contour;
//            }
//        }
//        Rect boundingRect = Imgproc.boundingRect(largestContour);
//        Imgproc.rectangle(mask, boundingRect, new Scalar(255, 0, 0), 3);


//        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2RGB);
//        Core.bitwise_not(input, input, mask);
//        Core.bitwise_not(input, input, mask);
//        Core.bitwise_and(input, mask, input);
//162, 100, 42
//255, 210, 137
//149, 73, 30

//HSV
//37, 46.3%, 100%
//22, 79.9%, 58.4%

//        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        kernel.release();
        cannyEdges.release();

        return input;
    }

    public boolean isCubeClose(){
        return cubeClose;
    }

    public boolean isDuckClose(){
        return duckClose;
    }

    public double getCubeXValueCommand(){
        if(cubeXValueCommandGood){
            return cubeXValueCommand;
        } else {
            throw new RuntimeException();
        }
    }
}
