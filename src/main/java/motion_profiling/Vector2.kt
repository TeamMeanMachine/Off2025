package org.team2471.frc.lib.math

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import java.math.BigDecimal
import java.math.RoundingMode

data class Vector2(var x: Double, var y: Double) /*: Interpolable<Vector2>*/ {
//    val angle: Angle get() = Math.atan2(x, y).radians

//    override fun toString(): String {
//        return "(${round(x, 7)}, ${round(y, 7)})"
//    }

    fun round(decimalPlaces: Int = 0): Vector2 {
        return Vector2(BigDecimal(this.x).setScale(decimalPlaces, RoundingMode.HALF_EVEN).toDouble(), BigDecimal(this.y).setScale(decimalPlaces, RoundingMode.HALF_EVEN).toDouble())
    }



    operator fun plus(b: Vector2) = Vector2(x + b.x, y + b.y)

    operator fun minus(b: Vector2) = Vector2(x - b.x, y - b.y)

    operator fun times(scalar: Double) = Vector2(x * scalar, y * scalar)

    operator fun div(scalar: Double) = Vector2(x / scalar, y / scalar)

    fun dot(b: Vector2) = (x * b.x) + (y * b.y)


    fun flipXAndY() = Vector2(y, x)

    fun perpendicular() = Vector2(y, -x)

    fun mirrorXAxis() = Vector2(-x, y)

    fun mirrorYAxis() = Vector2(x, -y)

    fun reflectAcrossField(middle: Double = 27.216667) = Vector2(middle * 2 - x, y)

    fun distance(other: Vector2) = Math.hypot(x - other.x, y - other.y)

    fun set(other: Vector2) {
        x = other.x
        y = other.y
    }

    fun set(X: Double, Y: Double) {
        x = X
        y = Y
    }

    fun coerceIn(otherMin: Vector2, otherMax: Vector2) {
        set(
            this.x.coerceIn(otherMin.x, otherMax.x),
            this.y.coerceIn(otherMin.y, otherMax.y)
        )
    }

//    override fun interpolate(other: Vector2, x: Double): Vector2 {
//        return when {
//            x <= 0.0 -> this
//            x >= 1.0 -> other
//            else -> Vector2(x * (other.x - this.x) + this.x, x * (other.y - this.y) + this.y)
//        }
//    }
}

fun Translation2d.asVector2() = Vector2(this.x, this.y)

fun Vector2.toTranslation2d(): Translation2d = Translation2d(this.x, this.y)

fun Vector2.toPose2d(heading: Double): Pose2d = Pose2d(this.toTranslation2d(), Rotation2d(heading))