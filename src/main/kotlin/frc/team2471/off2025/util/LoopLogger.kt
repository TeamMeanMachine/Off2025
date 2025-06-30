package frc.team2471.off2025.util

import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.Logger

object LoopLogger {
    private val prevTimes = mutableMapOf<String, Double>()
    private var startTime = Timer.getFPGATimestamp()

    fun reset() {
        startTime = Timer.getFPGATimestamp()
    }

    /** Log the period and the time of the function.  */
    fun record(loopName: String): Pair<Double, Double> {
        val now = Timer.getFPGATimestamp()
        val prevTime = prevTimes[loopName] ?: now
        val period = now - prevTime
        val sinceReset = now - startTime
        prevTimes[loopName] = now
        Logger.recordOutput("LoopLogger/$loopName/Period", period)
        Logger.recordOutput("LoopLogger/$loopName/SinceReset", sinceReset)
        return Pair(period, sinceReset)
    }
}