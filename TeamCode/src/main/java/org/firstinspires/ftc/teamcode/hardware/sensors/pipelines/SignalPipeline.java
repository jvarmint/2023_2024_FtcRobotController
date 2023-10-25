package org.firstinspires.ftc.teamcode.hardware.sensors.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class SignalPipeline extends OpenCvPipeline {
    Mat submat;
    Mat line = new Mat();
    Mat oMask = new Mat();
    Mat pMask = new Mat();
    Mat gMask = new Mat();
//    final Scalar oLower = new Scalar(15, 50, 50);//150, 90
//    final Scalar oUpper = new Scalar(28, 255, 255);// 230, 255
//    final Scalar pLower = new Scalar(170, 50, 50);//150, 90
//    final Scalar pUpper = new Scalar(200, 255, 255);// 230, 255
//    final Scalar gLower = new Scalar(70, 50, 50);//150, 90
//    final Scalar gUpper = new Scalar(90, 255, 255);// 230, 255
    final Scalar oLower = new Scalar(20, 50, 50);//150, 90
    final Scalar oUpper = new Scalar(40, 255, 250);// 230, 255
    final Scalar pLower = new Scalar(150, 50, 10);//150, 90
    final Scalar pUpper = new Scalar(200, 255, 255);// 230, 255
    final Scalar gLower = new Scalar(60, 50, 10);//150, 90
    final Scalar gUpper = new Scalar(90, 255, 255);// 230, 255
    Point position = new Point(50,50);
    private String signalColor = "";



    @Override
    public void init(Mat input) {

    }
//line from 325-326
    // purple: h=270
    // green: h=124
    // orange: h=31
    @Override
    public Mat processFrame(Mat input) {
        int gwhite = 0;
        int owhite = 0;
        int pwhite = 0;
        String winner;
        submat = input.submat(260,261,150,300);
        Imgproc.cvtColor(submat, line, Imgproc.COLOR_RGB2HSV_FULL);
        Core.inRange(line, oLower, oUpper, oMask);
        Core.inRange(line, pLower, pUpper, pMask);
        Core.inRange(line, gLower, gUpper, gMask);
        Mat rval = input;
        double[] looking = input.get(50,50);
        String color = "h: " + Integer.toString((int)(looking[0]*180.0/255.0)) + " s: " + Integer.toString((int)looking[1]) + " v: " + Integer.toString((int)looking[2]);
        Imgproc.putText(rval, color, new Point(100, 150), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(100,100,100), 3);

        Imgproc.circle(rval, position, 10, pUpper);

        for(int i = 0; i < submat.cols(); i++){
            owhite += (int)oMask.get(0,i)[0];
            gwhite += (int)gMask.get(0,i)[0];
            pwhite += (int)pMask.get(0,i)[0];
        }
        if(owhite>gwhite && owhite>pwhite){
            signalColor = "Orange";
        }
        else if(pwhite>owhite && pwhite>gwhite){
            signalColor = "Purple";
        }
        else{
            signalColor = "Green";
        }

        Imgproc.putText(rval,  "orange: "+ owhite+ " green: "+ gwhite + " purple: " + pwhite, new Point(100,50), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(100,100,100), 3);
        Imgproc.putText(rval,signalColor, new Point(100, 125), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(100,100,100), 3);

        Imgproc.rectangle(rval, new Rect(150,260, 150,1), new Scalar(0,255,0),2);

        return rval;

    }
    public int getCurrentSignal(){
        switch(signalColor){
            case "Purple":
                return 1;
            case "Green":
                return 2;
            case "Orange":
                return 3;
            default:
                return -1;
        }
    }
}
