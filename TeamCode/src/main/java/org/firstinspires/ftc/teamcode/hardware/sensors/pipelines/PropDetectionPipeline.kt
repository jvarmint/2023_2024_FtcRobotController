
package org.firstinspires.ftc.teamcode.hardware.sensors.pipelines

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.abs

class PropDetectionPipeline
    () : OpenCvPipeline() {

    private val redLower = Scalar(0.0, 200.0, 50.0) //v=100 works for the center part of the camera
    private val redUpper = Scalar(14.0, 230.0, 184.0)

    private val pinkLower = Scalar(0.0,50.0, 0.0 )//110 - 190 v
    private val pinkUpper = Scalar(14.0, 150.0, 255.0) //60-80 s

    private val lightBLower = Scalar(196.0, 170.0, 90.0)
    private val lightBUpper = Scalar(155.0, 186.0, 128.0)

    private val darkBLower = Scalar(145.0, 175.0, 60.0)
    private val darkBUpper = Scalar(158.0, 220.0, 115.0)

    private var hsv = Mat()
    private var maskR = Mat()
    private var maskP = Mat()
    private var maskLB = Mat()
    private var maskDB = Mat()
    private var verticalSquish = 30
    private var squished = Mat()

    private var topTargetError = 0.0
    private var midTargetError = 0.0
    private var lowTargetError = 0.0

    override fun processFrame(input: Mat): Mat {

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV_FULL)
        val c1=hsv.get(100,100)
        Imgproc.putText(input, ("H:"+c1[0]+" S"+c1[1]+" V"+c1[2]), Point (150.0, 150.0), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, Scalar(200.0, 100.0, 100.0), 3)
        Imgproc.circle(input, Point(100.0,100.0), 10, Scalar(255.0,255.0,255.0))
        val c2=hsv.get(200,300)
        Imgproc.putText(input, ("H:"+c2[0]+" S"+c2[1]+" V"+c2[2]), Point (350.0, 200.0), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, Scalar(200.0, 100.0, 100.0), 3)
        Imgproc.circle(input, Point(300.0,200.0), 10, Scalar(255.0,255.0,255.0))

        Core.inRange(hsv, redLower, redUpper, maskR)
        Core.inRange(hsv, pinkLower, pinkUpper, maskP)
        Core.inRange(hsv, lightBLower, lightBUpper, maskLB)
        Core.inRange(hsv, darkBLower, darkBUpper, maskDB)


//        val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(erodeScale*15.0, erodeScale*15.0))
//        Imgproc.erode(maskY, maskY, kernel)
//        Imgproc.dilate(maskY, maskY, kernel)

/////////////////////////////////////RED and BLUE erode/dialate/////////////////////////////
//        val kernelCones = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(70.0,1.0))
//        Imgproc.erode(maskCones,maskCones,kernelCones)
//
//        val kernelBlur = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(70.0,3.0))
//        Imgproc.dilate(maskCones,maskCones,kernelBlur)
///////////////////////////////////////////////////////////////////////////////////////////
//
//        Core.add(maskCones,maskY,maskCones)
//
//        maskCones = maskY
//
//        Imgproc.resize(maskCones,squished,squished.size(), 1.0, 1.0/verticalSquish)
//
//        val arrayList = ArrayList<Point>()
//
//        for (i in topCutoff/verticalSquish until squished.rows()) {
//            val midpoint = topTarget.x.toInt()
//
//            val positionOnRight = closestBlob(squished, i, midpoint, true)
//            val positionOnLeft = closestBlob(squished, i, midpoint, false)
//
//            val finalPosition = if(positionOnRight < 0 || positionOnLeft < 0){
//                ((abs(positionOnRight) + abs(positionOnLeft))/ 2)//midpoint case error still
//            } else if(positionOnLeft == Int.MAX_VALUE && positionOnRight == Int.MAX_VALUE){
//                -1
//            } else if(abs(positionOnRight-midpoint) < abs(midpoint-positionOnLeft)){
//                positionOnRight
//            } else {
//                positionOnLeft
//            }
//
//            if(finalPosition != -1) {
//                arrayList.add( Point(finalPosition.toDouble(), i.toDouble()*verticalSquish))
//            }
//        }
//
//        for (i in 0 until arrayList.size) {
//            Imgproc.circle(input, arrayList[i], 10,Scalar(200.0,100.0,100.0) )
//        }
//
//        var slopes = ArrayList<Double>()
//        var pointOnLine = Point()
//
//        for (i in 0 until arrayList.size-1){
//            val m = (arrayList[i+1].x - arrayList[i].x)/(arrayList[i+1].y - arrayList[i].y)
//            if(abs(m) <1.5) {
//                slopes.add(m)
//                pointOnLine.x += arrayList[i].x
//                pointOnLine.y += arrayList[i].y
//            }
//        }
//
//        var averageSlope = slopes.average()
//        pointOnLine.x /= arrayList.size-1
//        pointOnLine.y /= arrayList.size-1
//        Imgproc.circle(input, pointOnLine, 10,Scalar(100.0,100.0,200.0), 3)
//
//        val b = pointOnLine.x - (averageSlope * pointOnLine.y)
//
//        arrayList.sortBy{
//            abs(averageSlope*it.y + b - it.x)
//        }
//        pointOnLine=Point()
//        slopes=ArrayList<Double>()
//        val pointsToKeep = (0.9* (arrayList.size-1)).toInt()
//        for (i in 0 until pointsToKeep){
//            val m = (arrayList[i+1].x - arrayList[i].x)/(arrayList[i+1].y - arrayList[i].y)
//            if(abs(m) <1.5) {
//                slopes.add(m)
//                pointOnLine.x += arrayList[i].x
//                pointOnLine.y += arrayList[i].y
//            }
//        }
//        averageSlope = slopes.average()
//        pointOnLine.x /= pointsToKeep
//        pointOnLine.y /= pointsToKeep
//
//        val topPoint = Point(b, 0.0)
//        val bottomPoint = Point((averageSlope*input.rows())+b, input.rows().toDouble())
//
//        Imgproc.line(input, bottomPoint, topPoint, Scalar(100.0, 200.0, 100.0), 3)
//        Imgproc.line(input, Point(0.0, topTarget.y), Point (input.cols().toDouble(), topTarget.y),Scalar (100.0, 200.0, 100.0), 3)
//        Imgproc.line(input,topTarget, Point(topTarget.x,1000.0),Scalar(100.0,100.0,255.0),3)
//        val intersectionPoint = Point((averageSlope*topTarget.y)+b, topTarget.y)
//
//
//        topTargetError = topTarget.x - intersectionPoint.x
//
//        midTargetError = midTarget.x - (averageSlope*midTarget.y) + b
//
//        lowTargetError = lowTarget.x - (averageSlope*lowTarget.y) + b
//
//        //Cleanup
//        kernel.release()
//        kernelCones.release()
//        kernelBlur.release()
//
//        return maskY
//    }
//
//    fun distanceFromCenterHigh() : Double {
//        return topTargetError
//    }
//
//    fun distanceFromCenterMid() : Double {
//        return midTargetError
//    }
//
//    fun distanceFromCenterLow() : Double {
//        return lowTargetError
//    }
//
//    private fun closestBlob(input:Mat, heightOnImage:Int, midpoint:Int, isDirectionRight:Boolean):Int{
//
//        if(isDirectionRight){
//            for(i in midpoint until input.width()){
//                if(input[heightOnImage, i][0] >100){
//                    var lookin = 0
//                    while(input[heightOnImage, i+lookin][0] >100 && i+lookin<input.width()-1){
//                        lookin++
//                    }
//                    return (i+lookin/2) * if (input[heightOnImage,midpoint][0]>100) (-1) else (1)
//                }
//            }
//        }
//        else{
//            for(i in midpoint downTo  leftCutoff){
//                if(input[heightOnImage,i][0]>100){
//                    var lookin = 0
//                    while(input[heightOnImage, i+lookin][0]>100 && i+lookin>0){
//                        lookin--
//                    }
//                    return (i+lookin/2) * if (input[heightOnImage,midpoint][0]>100) (-1) else (1)
//                }
//            }
//        }
        return maskP;

    }
}