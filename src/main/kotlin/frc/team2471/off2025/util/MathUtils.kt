package frc.team2471.off2025.util

import edu.wpi.first.math.geometry.Twist2d
import kotlin.math.*

fun square(x: Double): Double = x.square()

fun Double.square(): Double = this * this

fun Double.squareWithSign() = (this.square()).withSign(this)

fun Double.cube() = this * this * this

fun Double.squareRootWithSign() = sqrt(abs(this)).withSign(this)

fun Double.log10() = log10(this)

fun average(vararg x: Double) = x.sum() / x.size

fun lerp(min: Double, max: Double, k: Double) = min + (max - min) * k

fun Double.deadband(tolerance: Double): Double = if (abs(this) < tolerance) {
    0.0
} else {
    (this - tolerance.withSign(this)) / (1.0 - tolerance)
}

// doesn't work with negative values of n
infix fun Double.mod(n: Double) = if (this < 0) {
    (this % n + n) % n
} else {
    this % n
}

fun Double.round(digits: Int): Double {
    return round(this, digits)
}
@JvmName("roundToDigit")
fun round(number: Double, digits: Int): Double {
    if (!number.isNaN()) {
        val modulo = 10.0.pow(digits.toDouble())
        return (number * modulo).roundToInt() / modulo
    }
    else {
        return number
    }
}

fun linearMap(inLo: Double, inHi: Double, outLo: Double, outHi: Double, inAlpha: Double): Double {
    return (inAlpha - inLo) / (inHi - inLo) * (outHi - outLo) + outLo
}

fun cubicMap(inLo: Double, inHi: Double, outLo: Double, outHi: Double, inAlpha: Double): Double {
    val x = (inAlpha - inLo) / (inHi - inLo)
    val cubic = (3 - 2 * x) * x * x
    return cubic * (outHi - outLo) + outLo
}

fun windRelativeAngles(angle1: Double, angle2: Double): Double {
    val diff = angle1 - angle2
    val absDiff = abs(diff)
    return if (absDiff > 180.0) {
        angle2 + 360.0 * sign(diff) * floor((absDiff / 360.0) + 0.5)
    } else {
        angle2
    }
}

fun interpTo(from: Double, to: Double, speed: Double, dt: Double = 0.02): Double {
    return from + (to - from) * dt * speed
}

fun epsilonEquals(a: Double, b: Double, epsilon: Double): Boolean {
    return (a - epsilon <= b) && (a + epsilon >= b)
}

fun epsilonEquals(a: Double, b: Double): Boolean {
    return epsilonEquals(a, b, 1e-5)
}

fun epsilonEquals(twist: Twist2d, other: Twist2d): Boolean {
    return epsilonEquals(twist.dx, other.dx) &&
            epsilonEquals(twist.dy, other.dy) &&
            epsilonEquals(twist.dtheta, other.dtheta)
}

fun Twist2d.epsilonEquals(other: Twist2d) = epsilonEquals(this, other)

fun Double.epsonEquals(other: Double) = epsilonEquals(this, other)
