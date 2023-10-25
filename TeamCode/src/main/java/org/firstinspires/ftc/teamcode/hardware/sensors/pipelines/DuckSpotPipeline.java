package org.firstinspires.ftc.teamcode.hardware.sensors.pipelines;

import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_SIMPLEX;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class DuckSpotPipeline extends OpenCvPipeline {
    Mat grey = new Mat();
    Mat hsv = new Mat();
    Mat mask = new Mat();
    Mat hierarchy = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    volatile int duckPos = -1;
    final int WIDTH = 640;
    final int HEIGHT = 480;
    final Scalar lower = new Scalar(4, 50, 20);//150, 90
    final Scalar upper = new Scalar(36, 255, 255);// 230, 255


//99

    //217
    //321


    @Override
    public Mat processFrame(Mat input) {
//        final Scalar lower = new Scalar(0, 0, 20);//188.1, 148.92);
//        final Scalar upper = new Scalar(50, 255, 250);//203.7, 255);

        Mat nonCroppedHsv = new Mat();
        Imgproc.cvtColor(input, nonCroppedHsv, Imgproc.COLOR_RGB2HSV);

        Rect rectCrop = new Rect(0, (int) (nonCroppedHsv.height()/2.0), (int) (nonCroppedHsv.width()*2.0/3.0), (int) (nonCroppedHsv.height()/2.0));
        hsv = new Mat(nonCroppedHsv, rectCrop);

        Core.inRange(hsv, lower, upper, mask);


//        grey.empty();
//        Core.add(grey, new Scalar(255, 0, 0), grey);
//        input.copyTo(grey, mask);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.erode(mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);

        Mat cannyEdges = new Mat();
        Imgproc.Canny(mask, cannyEdges, 10, 100);

//        MatOfPoint totalContours = new MatOfPoint();
        List<MatOfPoint> tempContours = new ArrayList<>();
        Imgproc.findContours(cannyEdges, tempContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        contours = tempContours;
        double maxArea = 0;
        MatOfPoint largestContour = new MatOfPoint();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }
        Rect boundingRect = Imgproc.boundingRect(largestContour);
        //99 left
        //217 middle
        //321 right
        if(boundingRect.x < 150){
            duckPos = 0;
        } else if (boundingRect.x < 275){
            duckPos = 1;
        } else {
            duckPos = 2;
        }

//        for(int i =0; i<contours.size(); i++) {
//            totalContours.push_back(contours.get(i));
//        }
//        List<Point> listOfPoint = totalContours.toList();
//        Collections.sort(listOfPoint, new HeightComparator());
//        MatOfPoint totalContours = new MatOfPoint();
//        for (int i = 0; i < contours.size(); i++) {
//            totalContours.push_back(contours.get(i));
//        }
//        boolean duckFound = false;
//        try {
//            List<Point> listOfPoint = totalContours.toList();
//            Collections.sort(listOfPoint, new HeightComparator());
////            double scalarY = 1.0-(listOfPoint.get(0).y/((double)HEIGHT));
//            cubeXValueCommand =  0.1*(((listOfPoint.get(0).x+listOfPoint.get(1).x) / 2.0) / (WIDTH/2.0) - 1.0);
//            cubeXValueCommandGood = true;
//
//            if(possibleDuck){
//                for(int i = listOfPoint.size()-1; i>=0; i--){
//                    if(listOfPoint.get(i).y > HEIGHT/2.0){
//                        duckFound = listOfPoint.get(i).y > HEIGHT*0.75;
//                        break;
//                    }
//                }
//            }
//
//        } catch (Exception e){
//            cubeXValueCommandGood = false;
//        }
//        duckClose = duckFound;


        for(int i =0; i<contours.size(); i++) {
            Imgproc.drawContours(hsv, contours, i, new Scalar(100, 50, 100), 5);
        }
        Imgproc.putText(hsv, Double.toString(boundingRect.x) + ":" + Double.toString(boundingRect.y), new Point(100, 100), FONT_HERSHEY_SIMPLEX, 5, new Scalar(100, 0, 100), 4);
//        double maxArea = 0
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
        nonCroppedHsv.release();

        return hsv;
    }
    public int getDuckPos(){
        return duckPos;
    }

}
