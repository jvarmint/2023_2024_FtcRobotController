package org.firstinspires.ftc.teamcode.utilities

import kotlin.math.abs

class Vector2D {
    @JvmField
    var x = 0.0
    @JvmField
    var y = 0.0

    constructor() {}
    constructor(x: Double, y: Double) {
        this.x = x
        this.y = y
    }

    constructor(v: Vector2D) {
        set(v)
    }

    operator fun set(x: Double, y: Double) {
        this.x = x
        this.y = y
    }

    fun set(v: Vector2D) {
        x = v.x
        y = v.y
    }

    fun setZero() {
        x = 0.0
        y = 0.0
    }

    val components: DoubleArray
        get() = doubleArrayOf(x, y)
    val length: Double
        get() = Math.sqrt(x * x + y * y)
    val lengthSq: Double
        get() = x * x + y * y

    fun distanceSq(vx: Double, vy: Double): Double {
        var vx = vx
        var vy = vy
        vx -= x
        vy -= y
        return vx * vx + vy * vy
    }

    fun distanceSq(v: Vector2D): Double {
        val vx = v.x - x
        val vy = v.y - y
        return vx * vx + vy * vy
    }

    fun distance(vx: Double, vy: Double): Double {
        var vx = vx
        var vy = vy
        vx -= x
        vy -= y
        return Math.sqrt(vx * vx + vy * vy)
    }

    fun distance(v: Vector2D): Double {
        val vx = v.x - x
        val vy = v.y - y
        return Math.sqrt(vx * vx + vy * vy)
    }

    val angle: Double
        get() = Math.atan2(y, x)

    fun normalize() {
        val magnitude = length
        x /= magnitude
        y /= magnitude
    }

    fun normalizeNotZero() {
        if (x != 0.0 && y != 0.0) {
            val magnitude = length
            x /= magnitude
            y /= magnitude
        }
    }

    val normalized: Vector2D
        get() {
            val magnitude = length
            return Vector2D(x / magnitude, y / magnitude)
        }

    //I did this so it could be wrong
    val normalizedSquare: Vector2D
        get() { //I did this so it could be wrong
            if (x == 0.0 && y == 0.0) return Vector2D()
            val magnitude = Math.max(Math.abs(x), Math.abs(y))
            return Vector2D(x / magnitude, y / magnitude)
        }

    fun normalizeSquare() {
        if (x != 0.0 && y != 0.0) {
            val magnitude = Math.max(Math.abs(x), Math.abs(y))
            x /= magnitude
            y /= magnitude
        }
    }

    fun clampSquareSmaller() {
        val max = Math.max(abs(x), abs(y))
        if (max > 1) {
            x /= max
            y /= max
        }
    }

    fun add(v: Vector2D) {
        x += v.x
        y += v.y
    }

    fun add(vx: Double, vy: Double) {
        x += vx
        y += vy
    }

    fun getAdded(v: Vector2D): Vector2D {
        return Vector2D(x + v.x, y + v.y)
    }

    operator fun plus(v: Vector2D): Vector2D {
        return Vector2D(x + v.x, y + v.y)
    }

    fun subtract(v: Vector2D) {
        x -= v.x
        y -= v.y
    }

    fun subtract(vx: Double, vy: Double) {
        x -= vx
        y -= vy
    }

    fun getSubtracted(v: Vector2D): Vector2D {
        return Vector2D(x - v.x, y - v.y)
    }

    operator fun minus(v: Vector2D): Vector2D {
        return Vector2D(x - v.x, y - v.y)
    }

    fun multiply(scalar: Double) {
        x *= scalar
        y *= scalar
    }

    operator fun times(scalar: Double):Vector2D {
        return Vector2D(x*scalar, y*scalar)
    }

    fun getMultiplied(scalar: Double): Vector2D {
        return Vector2D(x * scalar, y * scalar)
    }

    fun divide(scalar: Double) {
        x /= scalar
        y /= scalar
    }

    fun getDivided(scalar: Double): Vector2D {
        return Vector2D(x / scalar, y / scalar)
    }

    operator fun div(scalar: Double): Vector2D {
        return Vector2D(x / scalar, y / scalar)
    }
    val perp: Vector2D
        get() = Vector2D(-y, x)

    fun dot(v: Vector2D): Double {
        return x * v.x + y * v.y
    }

    fun dot(vx: Double, vy: Double): Double {
        return x * vx + y * vy
    }

    fun cross(v: Vector2D): Double {
        return x * v.y - y * v.x
    }

    fun cross(vx: Double, vy: Double): Double {
        return x * vy - y * vx
    }

    fun project(v: Vector2D): Double {
        return this.dot(v) / length
    }

    fun project(vx: Double, vy: Double): Double {
        return this.dot(vx, vy) / length
    }

    fun getProjectedVector(v: Vector2D): Vector2D {
        return normalized.getMultiplied(this.dot(v) / length)
    }

    fun getProjectedVector(vx: Double, vy: Double): Vector2D {
        return normalized.getMultiplied(this.dot(vx, vy) / length)
    }

    //    public static double PerpendicularDistance(Vector2D currPoint, Vector2D endPoint, Vector2D prePoint){
    //        double worldDist = MathUtil.Distance(currPoint.x,currPoint.y, endPoint.x,endPoint.y);
    //        double lineDist = MathUtil.Distance(prePoint.x,prePoint.y, endPoint.x,endPoint.y);
    //        Vector2D worldPoint = new Vector2D(currPoint.x-endPoint.x,currPoint.y-endPoint.y);
    //        Vector2D linePoint = new Vector2D(prePoint.x-endPoint.x,prePoint.y-endPoint.y);
    //        double dotProduct = Vector2D.dot(worldPoint,linePoint);
    //        double theta = Math.acos(dotProduct/(worldDist*lineDist));
    //        return worldDist*(Math.sin(theta));
    //    }
    //    public static Vector2D LineHypotIntersect(Vector2D currPoint, Vector2D endPoint, Vector2D prePoint, double radius){
    //        Vector2D worldVector = new Vector2D(currPoint.x-prePoint.x,currPoint.y-prePoint.y);
    //        Vector2D endVector = new Vector2D(endPoint.x-prePoint.x,endPoint.y-prePoint.y);
    //        double perpMag = Vector2D.project(endVector,worldVector);
    //        Vector2D intersectVector = endVector.getNormalized().getMultiplied(perpMag);
    //        double a = intersectVector.distance(worldVector);
    //        double b = Math.sqrt(Math.pow(radius,2)-Math.pow(a,2) ); // Abs could be totally wrong here, but avoiding NaN
    //        //double b = Math.sqrt(Math.abs( Math.pow(radius,2)-Math.pow(a,2) )); // Abs could be totally wrong here, but avoiding NaN
    //        Vector2D normalizedEndVector = endVector.getNormalized();
    //        Vector2D rvalUnnormalized = normalizedEndVector.getMultiplied(b).getAdded(intersectVector).getSubtracted(worldVector);
    //        Vector2D rval = rvalUnnormalized.getNormalized();
    //        System.out.print("normalizedEndVector: ");System.out.println(normalizedEndVector);
    //        System.out.print("b: ");System.out.println(b);
    //        System.out.print("intersectVector: ");System.out.println(intersectVector);
    //        System.out.print("worldVector: ");System.out.println(worldVector);
    //        System.out.print("rvalUnnormalized: ");System.out.println(rvalUnnormalized);
    //        System.out.print("rval: ");System.out.println(rval);
    //        return rval;
    //    }
    //    public static Vector2D VectorProgressDeg(Vector2D preVector, double progress, double degrees){
    //        return preVector.getRotatedBy(progress*degrees);
    //    }
    fun rotateBy(angle: Double) {
        val cos = Math.cos(angle)
        val sin = Math.sin(angle)
        val rx = x * cos - y * sin
        y = x * sin + y * cos
        x = rx
    }

    fun getRotatedBy(angle: Double): Vector2D {
        val cos = Math.cos(angle)
        val sin = Math.sin(angle)
        return Vector2D(x * cos - y * sin, x * sin + y * cos)
    }

    fun rotateTo(angle: Double) {
        set(toCartesian(length, angle))
    }

    fun getRotatedTo(angle: Double): Vector2D {
        return toCartesian(length, angle)
    }

    fun reverse() {
        x = -x
        y = -y
    }

    val reversed: Vector2D
        get() = Vector2D(-x, -y)

    fun clone(): Vector2D {
        return Vector2D(x, y)
    }

    override fun equals(obj: Any?): Boolean {
        if (obj === this) {
            return true
        }
        if (obj is Vector2D) {
            val v = obj
            return x == v.x && y == v.y
        }
        return false
    }

    override fun toString(): String {
        return "[%.5f : %.5f]".format(x, y)
    }

    override fun hashCode(): Int {
        var result = x.hashCode()
        result = 31 * result + y.hashCode()
        return result
    }

    companion object {
        fun angleDifferenceDeg(vector1: Vector2D, vector2: Vector2D): Double {
            val rVal = Math.toDegrees(vector1.normalized.angle - vector2.normalized.angle)
            return if (rVal > 180.0) {
                rVal - 360.0
            } else if (rVal < -180.0) {
                rVal + 360.0
            } else {
                rVal
            }
        }

        @JvmStatic
        fun angleDifferenceDeg(vector: Vector2D, heading: Double): Double {
            var rVal = 90 - Math.toDegrees(vector.normalized.angle) - heading
            rVal %= 360.0
            return if (rVal > 180.0) {
                rVal - 360.0
            } else if (rVal < -180.0) {
                rVal + 360.0
            } else {
                rVal
            }
        }

        fun toCartesian(magnitude: Double, angle: Double): Vector2D {
            return Vector2D(magnitude * Math.cos(angle), magnitude * Math.sin(angle))
        }

        fun add(v1: Vector2D, v2: Vector2D): Vector2D {
            return Vector2D(v1.x + v2.x, v1.y + v2.y)
        }

        fun subtract(v1: Vector2D, v2: Vector2D): Vector2D {
            return Vector2D(v1.x - v2.x, v1.y - v2.y)
        }

        fun dot(v1: Vector2D, v2: Vector2D): Double {
            return v1.x * v2.x + v1.y * v2.y
        }

        fun cross(v1: Vector2D, v2: Vector2D): Double {
            return v1.x * v2.y - v1.y * v2.x
        }

        fun project(v1: Vector2D, v2: Vector2D): Double {
            return dot(v1, v2) / v1.length
        }

        fun getProjectedVector(v1: Vector2D, v2: Vector2D): Vector2D {
            return v1.normalized.getMultiplied(dot(v1, v2) / v1.length)
        }
    }
}