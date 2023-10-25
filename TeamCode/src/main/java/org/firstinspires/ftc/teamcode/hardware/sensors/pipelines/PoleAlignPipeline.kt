package org.firstinspires.ftc.teamcode.hardware.sensors.pipelines

import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.abs

class PoleAlignPipeline
    (
    private var topTarget: Point,
    private var midTarget: Point,
    private var lowTarget: Point,
    private var erodeScale: Double = 1.0,
    private var topCutoff: Int = 0,
    private var leftCutoff: Int = 0,
    private var isLeft: Boolean,
) : OpenCvPipeline() {

    private val rLower = Scalar(0.0, 30.0, 20.0) //150, 90
    private val rUpper = Scalar(10.0, 255.0, 250.0) // 230, 255

    private val rLowerHigh = Scalar(245.0, 30.0, 20.0) //150, 90
    private val rUpperHigh = Scalar(255.0, 255.0, 250.0) // 230, 255

    private val bLower = Scalar(127.0, 30.0, 50.0) //150, 90
    private val bUpper = Scalar(170.0, 255.0, 250.0) // 230, 255

    private val yLower = Scalar(20.0, 50.0, 50.0) //150, 90
    private val yUpper = Scalar(40.0, 255.0, 254.0) // 230, 255


    private var hsv = Mat()
    private var maskR = Mat()
    private var maskRHigh = Mat()
    private var maskB = Mat()
    private var maskY = Mat()
    private var maskCones = Mat()
    private var verticalSquish = 30
    private var squished = Mat()

    private var topTargetError = 0.0
    private var midTargetError = 0.0
    private var lowTargetError = 0.0

    override fun processFrame(input: Mat): Mat {

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV_FULL)

        Core.inRange(hsv, rLower, rUpper, maskR)
        Core.inRange(hsv, rLowerHigh, rUpperHigh, maskRHigh)
        Core.add(maskR, maskRHigh, maskR)
        Core.inRange(hsv, bLower, bUpper, maskB)
        Core.inRange(hsv, yLower, yUpper, maskY)

        //Core.add(maskR, maskB, maskCones) <-------Adding red and blue to total mask
        //Core.add(mask, maskY, mask)
        //maskCones = maskR


        val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(erodeScale*15.0, erodeScale*15.0))
        Imgproc.erode(maskY, maskY, kernel)
        Imgproc.dilate(maskY, maskY, kernel)

/////////////////////////////////////RED and BLUE erode/dialate/////////////////////////////
        val kernelCones = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(70.0,1.0))
//        Imgproc.erode(maskCones,maskCones,kernelCones)
//
        val kernelBlur = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(70.0,3.0))
//        Imgproc.dilate(maskCones,maskCones,kernelBlur)
///////////////////////////////////////////////////////////////////////////////////////////

//        Core.add(maskCones,maskY,maskCones)

        maskCones = maskY

        Imgproc.resize(maskCones,squished,squished.size(), 1.0, 1.0/verticalSquish)

        val arrayList = ArrayList<Point>()

        for (i in topCutoff/verticalSquish until squished.rows()) {
            val midpoint = topTarget.x.toInt()

            val (positionOnRight, rightWidth) = closestBlob(squished, i, midpoint, true)
            val (positionOnLeft, leftWidth) = closestBlob(squished, i, midpoint, false)

            val finalPosition = if(positionOnRight < 0 || positionOnLeft < 0){
                Imgproc.putText(input,
                    "$rightWidth : $leftWidth", Point (20.0, 100.0), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, Scalar(200.0, 100.0, 100.0), 3)
                (topTarget.x+leftWidth+(rightWidth-leftWidth) / 2)//midpoint case error still
            } else if(positionOnLeft == Int.MAX_VALUE && positionOnRight == Int.MAX_VALUE){
                -1
            } else if(abs(positionOnRight-midpoint) < abs(midpoint-positionOnLeft)){
                positionOnRight
            } else {
                positionOnLeft
            }

            if(finalPosition != -1) {
                arrayList.add( Point(finalPosition.toDouble(), i.toDouble()*verticalSquish))
            }
        }
        if(arrayList.size >= 2) {


            for (i in 0 until arrayList.size) {
                Imgproc.circle(input, arrayList[i], 10,Scalar(200.0,100.0,100.0) )
            }

            var slopes = ArrayList<Double>()
            var pointOnLine = Point()

            for (i in 0 until arrayList.size-1){
                val m = (arrayList[i+1].x - arrayList[i].x)/(arrayList[i+1].y - arrayList[i].y)
                if(abs(m) <1.5) {
                    slopes.add(m)
                    pointOnLine.x += arrayList[i].x
                    pointOnLine.y += arrayList[i].y
                }
            }

            var averageSlope = slopes.average()
            pointOnLine.x /= arrayList.size-1
            pointOnLine.y /= arrayList.size-1
            Imgproc.circle(input, pointOnLine, 10,Scalar(100.0,100.0,200.0), 3)

            val b = pointOnLine.x - (averageSlope * pointOnLine.y)

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

            val topPoint = Point(b, 0.0)
            val bottomPoint = Point((averageSlope*input.rows())+b, input.rows().toDouble())

            Imgproc.line(input, bottomPoint, topPoint, Scalar(100.0, 200.0, 100.0), 3)
            Imgproc.line(input, Point(0.0, topTarget.y), Point (input.cols().toDouble(), topTarget.y),Scalar (100.0, 200.0, 100.0), 3)
            Imgproc.line(input,topTarget, Point(topTarget.x,1000.0),Scalar(100.0,100.0,255.0),3)
            val intersectionPoint = Point((averageSlope*topTarget.y)+b, topTarget.y)


            topTargetError = topTarget.x - intersectionPoint.x

            midTargetError = midTarget.x - (averageSlope*midTarget.y) + b

            lowTargetError = lowTarget.x - (averageSlope*lowTarget.y) + b

        } else {
            topTargetError = 0.0
            midTargetError = 0.0
            lowTargetError = 0.0
        }
        //Cleanup
        kernel.release()
        kernelCones.release()
        kernelBlur.release()

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

    fun getImage(){

    }

    private fun closestBlob(input:Mat, heightOnImage:Int, midpoint:Int, isDirectionRight:Boolean):Pair<Int, Int>{

        if(isDirectionRight){
            for(i in midpoint until input.width()){
                if(input[heightOnImage, i][0] >100){
                    var lookin = 0
                    while(input[heightOnImage, i+lookin][0] >100 && i+lookin<input.width()-1){
                        lookin++
                    }
                    return Pair((i+lookin/2) * if (input[heightOnImage,midpoint][0]>100) (-1) else (1), lookin)
                }
            }
        }
        else{
            for(i in midpoint downTo  leftCutoff){
                if(input[heightOnImage,i][0]>100){
                    var lookin = 0
                    while(input[heightOnImage, i+lookin][0]>100 && i+lookin>0){
                        lookin--
                    }
                    return Pair((i+lookin/2) * if (input[heightOnImage,midpoint][0]>100) (-1) else (1), lookin)
                }
            }
        }
        return Pair(Int.MAX_VALUE, 0);
    }
}