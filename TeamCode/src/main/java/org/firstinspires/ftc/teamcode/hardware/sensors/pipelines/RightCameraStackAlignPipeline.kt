package org.firstinspires.ftc.teamcode.hardware.sensors.pipelines

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.abs


class RightCameraStackAlignPipeline : OpenCvPipeline() {

    private val rLower = Scalar(0.0, 30.0, 20.0) //150, 90
    private val rUpper = Scalar(10.0, 255.0, 255.0) // 230, 255

    private val rLowerHigh = Scalar(245.0, 30.0, 20.0) //150, 90
    private val rUpperHigh = Scalar(255.0, 255.0, 255.0) // 230, 255

    private val bLower = Scalar(127.0, 30.0, 50.0) //150, 90
    private val bUpper = Scalar(170.0, 255.0, 254.0) // 230, 255

    private val yLower = Scalar(20.0, 50.0, 50.0) //150, 90
    private val yUpper = Scalar(40.0, 255.0, 250.0) // 230, 255


    private var hsv = Mat()
    private var mask = Mat()
    private var maskR = Mat()
    private var maskRHigh = Mat()
    private var maskB = Mat()
    private var maskY = Mat()
    private var maskCones = Mat()
    private var hierarchy = Mat()
    private var contours: List<MatOfPoint> = java.util.ArrayList()
    private var leftCutoff = 30
    private var verticalSquish = 30
    private var topCutoff = 80
    private var squished = Mat()
    private var intersectionHeight = 35.0

    private var horizontalPosition = 0
    private var topTarget = Point(146.5, 0.0)
    private var topTargetError = 0.0
    private var midTarget = Point(158.5, 300.0)
    private var midTargetError = 0.0
    private var lowTarget = Point(158.5, 500.0)
    private var lowTargetError = 0.0



    override fun processFrame(input: Mat): Mat {

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV_FULL)

        Core.inRange(hsv, rLower, rUpper, maskR)
        Core.inRange(hsv, rLowerHigh, rUpperHigh, maskRHigh)
        Core.add(maskR, maskRHigh, maskR)
        Core.inRange(hsv, bLower, bUpper, maskB)
        Core.inRange(hsv, yLower, yUpper, maskY)

        Core.add(maskR, maskB, maskCones)
        //Core.add(mask, maskY, mask)
        //maskCones = maskR


        val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(15.0, 15.0))
        Imgproc.erode(maskY, maskY, kernel)
        Imgproc.dilate(maskY, maskY, kernel)


        val kernelCones = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(70.0,1.0))
        Imgproc.erode(maskCones,maskCones,kernelCones)

        val kernelBlur = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(70.0,3.0))
        Imgproc.dilate(maskCones,maskCones,kernelBlur)

        Core.add(maskCones,maskY,maskCones)

        Imgproc.resize(maskCones,squished,squished.size(), 1.0, 1.0/verticalSquish)

        val arrayList = ArrayList<Point>()

        for (i in topCutoff/verticalSquish until squished.rows()) {
            var total = 0
            var totalNum = 0
            for (j in leftCutoff until squished.cols()) {
                val pixel: DoubleArray? = squished.get(i, j)
                if(pixel != null){
                    if(pixel[0] > 100){
                        total += j
                        totalNum++
                    }
                }


                }

            if(totalNum != 0) {
                arrayList.add( Point(total.toDouble() / totalNum.toDouble(), i.toDouble()*verticalSquish))
            }
        }



        for (i in 0 until arrayList.size) {
            Imgproc.circle(input, arrayList[i], 10,Scalar(200.0,100.0,100.0), 3 )
        }

        var slopes = ArrayList<Double>()
        var pointOnLine = Point()

        for (i in 0 until arrayList.size-1){
            val m = (arrayList[i+1].x - arrayList[i].x)/(arrayList[i+1].y - arrayList[i].y)
            if(Math.abs(m)<1.5) {
                slopes.add(m)
                pointOnLine.x += arrayList[i].x
                pointOnLine.y += arrayList[i].y
            }
        }

        var averageSlope = slopes.average()
        pointOnLine.x /= arrayList.size-1
        pointOnLine.y /= arrayList.size-1
        Imgproc.circle(input, pointOnLine, 10,Scalar(100.0,100.0,200.0) )


        val b = pointOnLine.x - (averageSlope * pointOnLine.y)
        //
        //SORTING
        //
        //Sorted set of points/////////////////////////////////////////////////////////////////////
        arrayList.sortBy{
            abs(averageSlope*it.y + b - it.x)
        }
        pointOnLine=Point()
        slopes=ArrayList<Double>()
        val pointsToKeep = (0.9* (arrayList.size-1)).toInt()
        for (i in 0 until pointsToKeep){
            val m = (arrayList[i+1].x - arrayList[i].x)/(arrayList[i+1].y - arrayList[i].y)
            if(abs(m) <1.5) {
                slopes.add(m)
                pointOnLine.x += arrayList[i].x
                pointOnLine.y += arrayList[i].y
            }
        }
        averageSlope = slopes.average()
        pointOnLine.x /= pointsToKeep
        pointOnLine.y /= pointsToKeep
        ///////////////////////////////////////////////////////////////////////////////////////////

        val topPoint = Point(b, 0.0)
        val bottomPoint = Point((averageSlope*input.rows())+b, input.rows().toDouble())



        Imgproc.line(input, bottomPoint, topPoint, Scalar(100.0, 200.0, 100.0), 3)
        Imgproc.line(input, Point(0.0, topTarget.y), Point (input.cols().toDouble(), topTarget.y),Scalar (100.0, 200.0, 100.0), 3)

        val intersectionPoint = Point((averageSlope*topTarget.y)+b, topTarget.y)
        //Imgproc.putText(input, intersectionPoint.toString(), Point (20.0, 100.0), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, Scalar(200.0, 100.0, 100.0), 3)

        topTargetError = topTarget.x - intersectionPoint.x

        midTargetError = midTarget.x - (averageSlope*midTarget.y) + b

        lowTargetError = lowTarget.x - (averageSlope*lowTarget.y) + b



//        val cannyEdges = Mat()
//        Imgproc.Canny(mask, cannyEdges, 10.0, 100.0)

//        MatOfPoint totalContours = new MatOfPoint();

//        MatOfPoint totalContours = new MatOfPoint();
//        val tempContours: List<MatOfPoint> = ArrayList()
//        Imgproc.findContours(cannyEdges, tempContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)
//        contours = tempContours
//        var maxArea = 0.0
////        var largestContour: MatOfPoint? = MatOfPoint()
//        var largestBox : Rect = Rect()
//        for (contour in contours) {
////            val area = Imgproc.contourArea(contour)
//            val boundingBox = Imgproc.boundingRect(contour)
//            val area = boundingBox.area()
//            if (area > maxArea) {
//                maxArea = area
//                largestBox = boundingBox
////                largestContour = contour
//            }
//        }
//        val boundingRect = Imgproc.boundingRect(largestContour)
//
//        Imgproc.rectangle(input, largestBox, Scalar(0.0, 255.0, 0.0), 2)
//
//        for (i in contours.indices) {
//            Imgproc.drawContours(input, contours, i, Scalar(255.0, 0.0, 0.0), 2)
//        }
//
//        horizontalPosition = largestBox.x + largestBox.width/2

        //Cleanup
        kernel.release()
        kernelCones.release()
        kernelBlur.release()
//        cannyEdges.release()
        return input
    }

    fun distanceFromCenterHigh() : Double {
        return topTargetError
    }

    fun distanceFromCenterMid() : Double {
        return midTargetError
    }

    fun distanceFromCenterLow() : Double {
        return lowTargetError
    }
}