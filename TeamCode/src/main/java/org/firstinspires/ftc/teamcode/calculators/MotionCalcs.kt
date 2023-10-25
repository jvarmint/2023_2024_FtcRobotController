package org.firstinspires.ftc.teamcode.calculators


import org.firstinspires.ftc.teamcode.calculators.Interfaces.MotionCalc
import org.firstinspires.ftc.teamcode.calculators.Interfaces.MoveData
import org.firstinspires.ftc.teamcode.utilities.Vector2D
import org.firstinspires.ftc.teamcode.utilities.Vector2D.Companion.project
import kotlin.math.pow

object MotionCalcs {

    fun MotionLambda(calcLambda: (MoveData) -> Vector2D, progressLambda: (MoveData) -> Double) : MotionCalc {
        return object: Interfaces.MotionCalc {
            override fun myProgress(d: MoveData): Double {
                return progressLambda(d)
            }

            override fun CalcMotion(d: MoveData): Vector2D {
                return calcLambda(d)
            }

        }
    }

    fun MotionLambda(calcLambda: (MoveData) -> Vector2D) : MotionCalc {
        return object: Interfaces.MotionCalc {
            override fun myProgress(d: MoveData): Double {
                return 0.0
            }

            override fun CalcMotion(d: MoveData): Vector2D {
                return calcLambda(d)
            }

        }
    }
    //This will always output a power on the x axis of the robot and a power on the y axis
    /**
     * This is a less common way to drive it implies that
     * when the joystick is pushed away the robot will go away
     * disregarding orientation
     * @return A vector on the joysticks coordinates rotated by the heading of the robot
     */
    @JvmStatic
    fun ObjectCentricJoystick(): MotionCalc {
        return object : MotionCalc {
            override fun CalcMotion(d: MoveData): Vector2D {
                val power = d.driver.ls()
                return power.getRotatedBy(Math.toRadians(-d.heading))
            }

            override fun myProgress(d: MoveData): Double {
                return 0.0
            }
        }
    }

    /**
     * This is the most common way to drive a robot when the
     * joystick is pushed forward the robot goes forward
     * @return A vector on the joysticks coordinates
     */
    @JvmStatic
    fun FieldCentricJoystick(driverOffsetDeg: Double): MotionCalc {
        return object : MotionCalc {
            var myProgress = 0.0
            override fun myProgress(d: MoveData): Double {
                return myProgress
            }

            override fun CalcMotion(d: MoveData): Vector2D {
                return d.driver.ls().getRotatedBy(Math.toRadians(driverOffsetDeg)).normalizedSquare
            }
        }
    }

    /**
     * All motion is described by curves
     * @param curveLength The length of the curve maybe in cm
     * @param curveFromHeading Using compass heading (clock wise from the top 0, 90, 180, 270)
     * This is the heading the robot should travel at the beginning of its travel
     * @param curveToHeading Using compass heading (clock wise from the top 0, 90, 180, 270)
     * This is the heading the robot should travel at the end of its travel
     * @return returns the progress of the Curve
     * @return returns a unit vector of the direction that is desired
     */
    fun CurveMotion(
        curveLength: Double,
        curveFromHeading: Double,
        curveToHeading: Double
    ): MotionCalc {
        return object : MotionCalc {
            private var myProgress = 0.0
            private var worldDist = 0.0
            private val curve =
                Vector2D(Vector2D.toCartesian(1.0, Math.toRadians(curveFromHeading)))

            override fun myProgress(d: MoveData): Double {
                return myProgress
            }

            override fun CalcMotion(d: MoveData): Vector2D {
                worldDist += d.wPos.distance(d.preWPos)
                myProgress = worldDist / curveLength
                val targetHeading = (curveToHeading - curveFromHeading) * myProgress
                //                d.debugData1 = targetHeading;
                return curve.getRotatedBy(
                    Math.PI / 2 - Math.toRadians(
                        targetHeading
                    )
                ).normalized
            }
        }
    }
    @JvmStatic
    fun pointSplineMotion(startTurnPercent : Double, vararg points: Vector2D) : MotionCalc {
        return object : MotionCalc {

            var initialized = false

            abstract inner class Segment {

                var t : Double = 0.0
                var distanceTraveled = 0.0
                var totalDistance: Double = 0.0

                abstract fun getCurrentPosition() : Vector2D
                abstract fun getCurrentVelocity() : Vector2D

                abstract fun getFixVelocity(worldPosition: Vector2D, p:Double): Vector2D

                abstract fun updateT(changeInDistance : Double): Double
            }

            inner class Curve(
                var startPoint: Vector2D,
                var controlPoint: Vector2D,
                var endPoint: Vector2D
            ) : Segment(){

                var lookUpTable = mutableMapOf<Double,Double>()

                init{

                    calculateLookUpTable()
                    totalDistance = lookUpTable.keys.last()
                }

                private fun calculateLookUpTable() {


                    val numIterations = 1000
                    var sum = 0.0

                    var previousVec = startPoint.clone()

                    for(i in 1..numIterations){
                        val t = i/numIterations.toDouble()
                        val currentVec = startPoint + (controlPoint-startPoint) * (3*t-3*t.pow(2)) + (endPoint-startPoint) * (t.pow(3))
                        sum += currentVec.distance(previousVec)
                        lookUpTable.put(sum, t)
                        previousVec = currentVec
                    }
                }

                override fun getCurrentPosition(): Vector2D {
                    return startPoint + (controlPoint - startPoint) * (3*t-3*t.pow(2)) + (endPoint - startPoint) * (t.pow(3))
                }
                /*
                a
                        .getAdded(b.getMultiplied(3*t-3*t.pow(2)))
                        .getAdded(c.getMultiplied(t.pow(3)))
                 */

                override fun getFixVelocity(worldPosition: Vector2D, p:Double): Vector2D {
                    val fixVelocity =
                        (getCurrentPosition()-worldPosition)*(p)
                    return if (fixVelocity.length > 1.0) {
                        fixVelocity.normalized
                    } else {
                        fixVelocity
                    }
                }
                override fun getCurrentVelocity() : Vector2D { // t might not be an accurate representation of reality
                    return ((endPoint-startPoint) * (t.pow(2)) - (controlPoint-startPoint) * (t) - (controlPoint-startPoint) * (t-1)) * 3.0
                }
                /*
                a.getMultiplied(2*t-t.pow(2))//-(t.pow(2)) - 2.0*t - 1.0)
                        .getAdded(b.getMultiplied(1.0-2.0*t))
                        .getAdded(c.getMultiplied(t.pow(2)))
                        .getMultiplied(3.0)
                 */


                override fun updateT(changeInDistance: Double): Double {
                    distanceTraveled += changeInDistance
                    var temp = getInterpolatedFromMap(distanceTraveled, lookUpTable)
                    t = temp.first
                    return temp.second
                }
            }

            inner class Straight(
                var startPoint: Vector2D, 
                var endPoint: Vector2D
            ) : Segment(){

                init {
                    totalDistance = endPoint.distance(startPoint)
                }

                override fun getCurrentPosition(): Vector2D {
                    return (endPoint - startPoint) * t + startPoint
                }

                override fun getFixVelocity(worldPosition: Vector2D, p:Double): Vector2D {
                    val fixVelocity =
                        (getCurrentPosition()-worldPosition)*(p)
                    return if (fixVelocity.length > 1.0) {
                        fixVelocity.normalized
                    } else {
                        fixVelocity
                    }
                }

                override fun getCurrentVelocity() : Vector2D {
//                    return Vector2D(.5,0.0)
                    return (endPoint - startPoint)
                }

                override fun updateT(changeInDistance: Double): Double {
                    distanceTraveled += changeInDistance
                    t = distanceTraveled/totalDistance
                    val extra = distanceTraveled-totalDistance
                    if(extra>0){
                        distanceTraveled=totalDistance
                        return extra
                    }
                    return 0.0
                }

            }

            var segments = ArrayList<Segment>()

            var currentSegment: Int = 0

            var myProgress: Double = 0.0

            override fun myProgress(d: MoveData): Double {

                if(initialized) {
                    myProgress = segments.sumOf{ it.distanceTraveled} / segments.sumOf{ it.totalDistance }
                }

                return myProgress
            }

            override fun CalcMotion(d: MoveData): Vector2D {
                if(!initialized){
                    //create segments
                    if(points.isEmpty()){
                        myProgress = 1.0
                        return Vector2D()
                    }
                    if(points.size > 1) {
                        segments.add(Straight(startPoint = d.wPos.clone(), endPoint = (points[0]-(d.wPos.clone())) * (startTurnPercent) + (d.wPos.clone())))//can be complicated but for the sake of getting it working
                        segments.add(Curve(
                            startPoint = (points[0] - (d.wPos.clone())) * startTurnPercent + d.wPos.clone(),
                            controlPoint = points[0],
                            endPoint = (points[1] - points[0]) * (1.0-startTurnPercent) + (points[0])))

                        for(i in 1..points.size-2){
                            segments.add(Straight(
                                startPoint = (points[i] - points[i-1]) * (1.0-startTurnPercent) + points[i-1],
                                endPoint = (points[i] - points[i-1]) * startTurnPercent + points[i-1]))
                            segments.add(Curve(
                                startPoint = (points[i] - points[i-1]) * startTurnPercent + points[i-1],
                                controlPoint = points[i],
                                endPoint = (points[i+1] - points[i]) * (1.0-startTurnPercent) + points[i]))
                        }

                        segments.add(Straight(

                            startPoint = (points.last() - points[points.size - 2]) * (1.0-startTurnPercent) + points[points.size - 2],
                            endPoint = points.last()))

                    } else {
                        segments.add(Straight(startPoint = d.wPos.clone(), endPoint = points[0]))
                    }




                    initialized = true
                }

//                val delta = d.wPos - d.preWPos
//
//                val currentVelocity = segments[currentSegment].getCurrentVelocity().normalized
//
//                segments[currentSegment].updateT(changeInDistance = project(currentVelocity, delta))
//
//                if(segments[currentSegment].t >= 1.0) {
//                    currentSegment++
//                }
                val delta = d.wPos - d.preWPos
// Get the velocity for the frame we have been working on
                val lastVelocity = segments[currentSegment].getCurrentVelocity().normalized
// updateT will figure out extra distance is remaining after t==1.0 and return it
                val remainderChangeInDistance = segments[currentSegment].updateT(changeInDistance = project(lastVelocity, delta))
                if(segments[currentSegment].t >= 1.0) {
                    currentSegment++
                    if(currentSegment>=segments.size){
                        return lastVelocity
                    }
                }
                if (remainderChangeInDistance > 0.0) {
                    // Move us along t in the new segment with whatever distance wasn't used to finish the previous one
                    segments[currentSegment].updateT(remainderChangeInDistance)
                }
// Get the velocity that we should be working on for the latest t, and possibly new segment
                val currentVelocity = segments[currentSegment].getCurrentVelocity().normalized


//                val perpendicularFixVelocity = getProjectedVector(currentVelocity.perp, segments[currentSegment].getFixVelocity(d.wPos, 6.0))

//                d.telemetry.addData("error", segments[currentSegment].getCurrentPosition() - (d.wPos))

//                currentVelocity = Vector2D(1.0, 0.0)-Vector2D(0.0,0.0)

                d.telemetry.addData("current velocity", currentVelocity)


                return currentVelocity + segments[currentSegment].getFixVelocity(d.wPos, 4.0)
            }

        }
    }
    // Returns a pair with the t value and the remainder of unused distance
    fun getInterpolatedFromMap(target: Double, theMap:Map<Double,Double>) : Pair<Double,Double>
    {
        val k = theMap.keys
        // Handle the ends of the data, returning non-zero remainder
        if (target <= k.first() ){
            return Pair(theMap.getValue(k.first()), target - k.first())
        }
        else if (target >= k.last()) {
            return Pair(theMap.getValue(k.last()), target - k.last())
        }
        // Do the binary search for the spot in the keys we want
        val b = k.toList().binarySearch(target)

        var answer: Double;
        if (b < 0)
        {
            // Negative means that we are between keys
            val lowKey = k.elementAt(-b - 2 )
            val highKey = k.elementAt(-b - 1 )
            val lowVal = theMap.getValue(lowKey)
            val highVal = theMap.getValue(highKey)
            // lerp
            answer = (highVal - lowVal) * (target - lowKey)/(highKey - lowKey) + lowVal
        }
        else
        {
            answer = theMap.getValue(k.elementAt(b))
        }
        return Pair(answer,0.0)
    }


    /**
     * PointMotion is used to give the robot cartesian coordinates that start at (0,0) at a decided corner of the field
     * @param turnRadius this determines how tight the turn between the points will be
     * @param points these are all of the positions that the robot will travel to
     * @return this returns a normalized vector to be comprehended by [org.firstinspires.ftc.teamcode.op.ComplexOp]
     */
//    @JvmStatic
//    fun PointMotion(turnFromDistance: Double, vararg points: Vector2D): MotionCalc {
//        val telemetryMap = OtherCalcs.TelemetryPosition()
//
//        return object : MotionCalc {
//            private var myProgress = 0.0 //this is what is returned for progress
//            private var firstLoop = true //firstLoop is for determining what the start position is
//            private var totalDist = 0.0 //the total distance the robot will need to travel
//            private val worldDist =
//                0.0 //the amount of distance traveled // the worldDist/totalDist = myProgress in rough terms
//            private val ePosArray =
//                ArrayList<Vector2D>() //this is the vector the robot is traveling towards
//            private val preEPosArray =
//                ArrayList<Vector2D>() //this is the vector the robot is coming from
//            private val segmentDataArray =
//                ArrayList<SegmentData>() //this is a list of necessary information about the segments
//            private var currentSegmentIndex = 0
//            // ^ is used for determining the distance traveled
//            /**
//             * calcWorldDists adds all of the different segments between all of the points
//             * to determine the total distance needed to travel this is used to find the progress throughout the calc
//             * @return the total amount of distance needed to travel
//             */
//            fun calcWorldDists(): Double {
//                var total = 0.0
//                var lastDist = 0.0
//                for (i in segmentDataArray.indices) {
//                    total += segmentDataArray[i].length
//                    segmentDataArray[i].worldStartDist = lastDist
//                    segmentDataArray[i].worldEndDist = total
//                    lastDist = total
//                }
//                return total
//            }
//
//            /**
//             * GetSegmentByWorldDist finds the segment based on distance traveled
//             * @param worldDist is the total amount of distance traveled to determine the segment
//             * @return a [SegmentData]
//             */
//            fun GetSegmentByWorldDist(worldDist: Double): SegmentData {
//                for (i in segmentDataArray.indices) {
//                    val candidate = segmentDataArray[i]
//                    if (worldDist >= candidate.worldStartDist &&
//                        worldDist <= candidate.worldEndDist
//                    ) {
//                        return segmentDataArray[i]
//                    }
//                }
//                // Return the last one if none are in range
//                return segmentDataArray[segmentDataArray.size - 1]
//            }
//
//            /**
//             * The data that is used for defining a segment
//             */
//            open inner class SegmentData {
//                var worldStartDist = 0.0
//                var worldEndDist = 0.0
//                var length = 0.0
//                var startDirection: Vector2D? = null
//                var startPos: Vector2D? = null
//                var endPos: Vector2D? = null
//            }
//
//            /**
//             * This is a simplified implementation of [SegmentData]
//             */
//            inner class StraightData(length: Double) : SegmentData() {
//                /**
//                 * StraightData sets the length for a segment
//                 * @param length this is the length of the segment
//                 */
//                init {
//                    this.length = length
//                }
//            }
//
//            /**
//             * CurveData includes the information for defining a curve
//             */
//            inner class CurveData(var arcAngle: Double) : SegmentData() {
//                var curveLength: Double = abs(0.0
////                    Math.PI * 2.0 * radius * (180 - abs(
////                        arcAngle
////                    )) / 360.0
//                )
//                var arcSegCoverDist = turnFromDistance
//
//
//                /**
//                 * This is an extension of [SegmentData] using only the arcAngle and
//                 * the other information provided in the parameters of PointMotion
//                 * @param arcAngle is the theta between two vectors
//                 */
//                init {
//                    length = curveLength
//                }
//            }
//
//            override fun myProgress(d: MoveData): Double {
//                return myProgress
//            }
//
//            override fun CalcMotion(d: MoveData): Vector2D {
//                //to initialize all of the end points
//                if (firstLoop) {
//                    //setting the first point to where the robot starts || this could also be a chosen starting position
//                    preEPosArray.add(0, d.wPos.clone())
//                    var lastCurve: CurveData? = null
//                    for (i in points.indices) {
//                        //copying all of the arguments from PointMotion into a this class// because I can
//                        ePosArray.add(i, points[i].clone())
//                        //setting the previous endpoint
//                        // i+1 pre instead of i-1 e avoids having to deal with null pointer exceptions
//                        preEPosArray.add(i + 1, ePosArray[i].clone())
//                    }
//                    for (i in points.indices) {
//
//                        val fullSegmentLength = ePosArray[i].distance(preEPosArray[i])
//                        val tempStraight: SegmentData = StraightData(fullSegmentLength)
//                        if (lastCurve != null) {
//                            // Remove the first portion of the straight due to the curve
//                            tempStraight.length -= lastCurve.arcSegCoverDist
//                        }
//                        tempStraight.startDirection =
//                            ePosArray[i].getSubtracted(preEPosArray[i]).normalized
//                        tempStraight.startPos = preEPosArray[i].clone()
//                        tempStraight.endPos =
//                            ePosArray[i].clone() // To be replaced if there is a curve
//                        segmentDataArray.add(tempStraight)
//
//                        // Add the curve info
//                        if (i < points.size - 1) {
//                            //EDITING HERE
////                            Vector2D preVec = new Vector2D(ePosArray.get(i).x - preEPosArray.get(i).x,
////                                    ePosArray.get(i).y - preEPosArray.get(i).y);
////                            Vector2D postVec = new Vector2D(ePosArray.get(i + 1).x - ePosArray.get(i).x,
////                                    ePosArray.get(i + 1).y - ePosArray.get(i).y);
//                            val preVec =
//                                Vector2D(ePosArray[i].getSubtracted(preEPosArray[i])) ////////////////////////////
//                            val postVec =
//                                Vector2D(ePosArray[i + 1].getSubtracted(ePosArray[i])) ////////////////////////////
//                            val arcAngDiff = Vector2D.angleDifferenceDeg(postVec, preVec)
//                            val tempCurve = CurveData(arcAngDiff)
//                            tempCurve.startDirection = tempStraight.startDirection
//                            tempCurve.startPos =
//                                tempCurve.startDirection?.getMultiplied(tempCurve.arcSegCoverDist)?.let {
//                                    ePosArray[i].getSubtracted(
//                                        it
//                                    )
//                                }
//                            tempCurve.endPos =
//                                tempCurve.startPos?.getRotatedBy(Math.toRadians(tempCurve.arcAngle))
//                            segmentDataArray.add(tempCurve)
//                            lastCurve = tempCurve
//                            tempStraight.length -= tempCurve.arcSegCoverDist
//                            tempStraight.endPos = tempCurve.startPos
//                        }
//                    }
//                    totalDist = calcWorldDists()
//                    d.debugData1 = totalDist
//                    //so it doesn't loop again //very important
//                    firstLoop = false
//                }
//                telemetryMap.CalcOther(d)
//                var currentSegment = segmentDataArray[currentSegmentIndex]
//                myProgress =
//                    (currentSegment.worldStartDist + d.wPos.distance(currentSegment.startPos)) / totalDist
//                //                double segmentProgress = (worldDist - currentSegment.worldStartDist)/currentSegment.length;
//                val segmentProgress =
//                    currentSegment.startPos!!.distance(d.wPos) / currentSegment.length
//                if (segmentProgress > .99) { ///changed from .97
//                    currentSegmentIndex++
//                    if (currentSegmentIndex >= segmentDataArray.size) {
//                        myProgress = 1.00
//                        return Vector2D()
//                    } else {
//                        currentSegment = segmentDataArray[currentSegmentIndex]
//                    }
//                }
//
//                //System.out.print("segmentProgress: ");System.out.println(segmentProgress);
//                return if (currentSegment is CurveData) {
//                    currentSegment.startDirection!!.getRotatedBy(
//                        segmentProgress * Math.toRadians(
//                            currentSegment.arcAngle
//                        )
//                    )
//                } else {
//                    currentSegment.endPos!!.getSubtracted(d.wPos).normalized
//                    //Alternative that doesn't use current location: rval = currentSegment.startDirection;
//                }
//            }
//        }
//    }

    //
    //    public static Interfaces.MotionCalc DriveTowardsDuckBlue(){
    //        return new Interfaces.MotionCalc() {
    //            double myProgress = 0.0;
    //            @Override
    //            public Vector2D CalcMotion(Interfaces.MoveData d) {
    //                //find the ducks offset from the center of the intake
    //
    //                int duckOffset  = d.robot.findDuckPipeline.duckOffset;
    //
    //                //if it will run into a wall it will move on
    //                if(d.wPos.x > 5.8 || d.wPos.x < 0.2 || d.wPos.y > 2.0 || (d.wPos.y < 0.3 && d.wPos.x < 0.4)){
    //                    myProgress = 1.0;
    //                }
    //                //if the duck offset is less than 20 it will go forward
    //                //the ducks offset to the left or right is proportional to speed in teh y direction
    //                if(duckOffset == -160) return new Vector2D(0.0, 1.0);
    //                return new Vector2D(Math.abs(duckOffset)<20?-1.0: 0.0, duckOffset/160.0);
    //            }
    //
    //            @Override
    //            public double myProgress(Interfaces.MoveData d) {
    //                return myProgress;
    //            }
    //        };
    //    }


//    fun PointMotionNoProgress(turnRadius: Double, vararg points: Vector2D): MotionCalc {
//        return object : MotionCalc {
//            private var myProgress = 0.0 //this is what is returned for progress
//            private var firstLoop = true //firstLoop is for determining what the start position is
//            private var totalDist = 0.0 //the total distance the robot will need to travel
//            private var worldDist =
//                0.0 //the amount of distance traveled // the worldDist/totalDist = to myProgress in rough terms
//            private val ePosArray =
//                ArrayList<Vector2D>() //this is the vector the robot is traveling towards
//            private val preEPosArray =
//                ArrayList<Vector2D>() //this is the vector the robot is coming from
//            private val segmentDataArray =
//                ArrayList<SegmentData>() //this is a list of necessary information about the segments
//            private var currentSegment: SegmentData? = null
//            // ^ is used for determining the distance traveled
//            /**
//             * CalcWorldDists adds all of the different segments between all of the points
//             * to determine the total distance needed to travel this is used to find the progress throughout the calc
//             * @return the total amount of distance needed to travel
//             */
//            fun CalcWorldDists(): Double {
//                var total = 0.0
//                var lastDist = 0.0
//                for (i in segmentDataArray.indices) {
//                    total += segmentDataArray[i].length
//                    segmentDataArray[i].worldStartDist = lastDist
//                    segmentDataArray[i].worldEndDist = total
//                    lastDist = total
//                }
//                return total
//            }
//
//            /**
//             * GetSegmentByWorldDist finds the segment based on distance traveled
//             * @param worldDist is the total amount of distance traveled to determine the segment
//             * @return a [SegmentData]
//             */
//            fun GetSegmentByWorldDist(worldDist: Double): SegmentData {
//                for (i in segmentDataArray.indices) {
//                    val candidate = segmentDataArray[i]
//                    if (worldDist >= candidate.worldStartDist &&
//                        worldDist <= candidate.worldEndDist
//                    ) {
//                        return segmentDataArray[i]
//                    }
//                }
//                // Return the last one if none are in range
//                return segmentDataArray[segmentDataArray.size - 1]
//            }
//
//            /**
//             * The data that is used for defining a segment
//             */
//            open inner class SegmentData {
//                var worldStartDist = 0.0
//                var worldEndDist = 0.0
//                var length = 0.0
//                var startDirection: Vector2D? = null
//                var startPos: Vector2D? = null
//                var endPos: Vector2D? = null
//            }
//
//            /**
//             * This is a simplified implementation of [SegmentData]
//             */
//            inner class StraightData(length: Double) : SegmentData() {
//                /**
//                 * StraightData sets the length for a segment
//                 * @param length this is the length of the segment
//                 */
//                init {
//                    this.length = length
//                }
//            }
//
//            /**
//             * CurveData includes the information for defining a curve
//             */
//            inner class CurveData(var arcAngle: Double) : SegmentData() {
//                var curveLength: Double
//                var arcSegCoverDist: Double
//                var radius = turnRadius
//
//                /**
//                 * This is an extension of [SegmentData] using only the arcAngle and
//                 * the other information provided in the parameters of PointMotion
//                 * @param arcAngle is the theta between two vectors
//                 */
//                init {
//                    arcSegCoverDist = Math.tan(Math.toRadians(arcAngle / 2)) * radius
//                    curveLength = Math.abs(Math.PI * 2.0 * radius * arcAngle / 360.0)
//                    length = curveLength
//                }
//            }
//
//            override fun myProgress(d: MoveData): Double {
//                return 0.0
//            }
//
//            override fun CalcMotion(d: MoveData): Vector2D {
//                //to initialize all of the end points
//                if (firstLoop) {
//                    //setting the first point to where the robot starts || this could also be a chosen starting position
//                    preEPosArray.add(0, d.wPos.clone())
//                    var lastCurve: CurveData? = null
//                    for (i in points.indices) {
//                        //copying all of the arguments from PointMotion into a this class// because I can
//                        ePosArray.add(i, points[i].clone())
//                        //setting the previous endpoint
//                        // i+1 pre instead of i-1 e avoids having to deal with null pointer exceptions
//                        preEPosArray.add(i + 1, ePosArray[i].clone())
//                    }
//                    for (i in points.indices) {
//                        val fullSegmentLength = ePosArray[i].distance(preEPosArray[i])
//                        val tempStraight: SegmentData = StraightData(fullSegmentLength)
//                        if (lastCurve != null) {
//                            // Remove the first portion of the straight due to the curve
//                            tempStraight.length -= lastCurve.arcSegCoverDist
//                        }
//                        tempStraight.startDirection =
//                            ePosArray[i].getSubtracted(preEPosArray[i]).normalized
//                        tempStraight.startPos = preEPosArray[i].clone()
//                        tempStraight.endPos =
//                            ePosArray[i].clone() // To be replaced if there is a curve
//                        segmentDataArray.add(tempStraight)
//
//                        // Add the curve info
//                        if (i < points.size - 1) {
//                            val preVec = Vector2D(
//                                ePosArray[i].x - preEPosArray[i].x,
//                                ePosArray[i].y - preEPosArray[i].y
//                            )
//                            val postVec = Vector2D(
//                                ePosArray[i + 1].x - ePosArray[i].x,
//                                ePosArray[i + 1].y - ePosArray[i].y
//                            )
//                            val arcAngDiff = Vector2D.angleDifferenceDeg(postVec, preVec)
//                            val tempCurve = CurveData(arcAngDiff)
//                            tempCurve.startDirection = tempStraight.startDirection.clone()
//                            tempCurve.startPos = ePosArray[i].getSubtracted(
//                                tempCurve.startDirection.getMultiplied(tempCurve.arcSegCoverDist)
//                            )
//                            tempCurve.endPos =
//                                tempCurve.startPos.getRotatedBy(Math.toRadians(tempCurve.arcAngle))
//                            segmentDataArray.add(tempCurve)
//                            lastCurve = tempCurve
//                            tempStraight.length -= tempCurve.arcSegCoverDist
//                            tempStraight.endPos = tempCurve.startPos.clone()
//                        }
//                    }
//                    totalDist = CalcWorldDists()
//                    currentSegment = segmentDataArray[0]
//                    //so it doesn't loop again //very important
//                    firstLoop = false
//                }
//                worldDist += d.wPos.distance(d.preWPos)
//                //Making a ratio of how much we have traveled over what we should travel to create progress
//                myProgress = worldDist / totalDist
//                val currentSegment = GetSegmentByWorldDist(worldDist)
//                val segmentProgress =
//                    (worldDist - currentSegment.worldStartDist) / currentSegment.length
//                //System.out.print("segmentProgress: ");System.out.println(segmentProgress);
//                var rval: Vector2D? = null
//                rval = if (currentSegment is CurveData) {
//                    currentSegment.startDirection!!.getRotatedBy(
//                        segmentProgress * Math.toRadians(
//                            currentSegment.arcAngle
//                        )
//                    )
//                } else {
//                    currentSegment.endPos!!.getSubtracted(d.wPos).normalized
//                    //Alternative that doesn't use current location: rval = currentSegment.startDirection;
//                }
//                return rval
//            }
//        }
//    }
//
//    fun ConstantDistanceToPoint(distance: Double, inputPoint: Vector2D): MotionCalc {
//        return object : MotionCalc {
//            override fun CalcMotion(d: MoveData): Vector2D {
//                val point = Vector2D(inputPoint)
//                val currDistance = d.wPos.distance(point)
//                val magnitude = currDistance - distance
//                point.subtract(d.wPos)
//                point.normalizeNotZero()
//                point.multiply(magnitude)
//                point.normalizeSquareSmaller()
//                val perp = Vector2D(inputPoint.clone())
//                perp.perp()
//                perp.normalizeNotZero()
//                perp.multiply(d.driver.ls().x)
//                point.add(perp)
//                return point
//            }
//
//            override fun myProgress(d: MoveData): Double {
//                return 0
//            }
//        }
//    }

    fun AlignPost(): MotionCalc {
        return object : MotionCalc {
            override fun CalcMotion(d: MoveData): Vector2D {
                val leftCameraDirection = Vector2D(1.0, 0.0)
                leftCameraDirection.rotateBy(Math.toRadians(-26.0))
                val rightCameraDirection = Vector2D(1.0, 0.0)
                rightCameraDirection.rotateBy(Math.toRadians(55.0))
                val leftError = d.robot.leftPoleAlignPipeline.distanceFromCenterHigh() / 120.0
                val rightError = d.robot.rightPoleAlignPipeline.distanceFromCenterHigh() / 120.0
                //                    if(leftError>.5)
//                        leftError=.5;
//                    if(rightError>.5)
//                        rightError=.5;
                val tempTotalVector = leftCameraDirection * (-leftError) +
                        rightCameraDirection * (-rightError)
                return if (tempTotalVector.length > .5) {
                    tempTotalVector.normalized / 2.0
                } else {
                    tempTotalVector
                }
            }

            override fun myProgress(d: MoveData): Double {
                return 0.0
            }
        }
    }
}