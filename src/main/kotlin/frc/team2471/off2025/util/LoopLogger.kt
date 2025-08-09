package frc.team2471.off2025.util

import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.Logger

object LoopLogger {
    private val prevTimes = mutableMapOf<String, Double>()
    private var startTime = Timer.getFPGATimestamp()
    private val loopsAndIndex = mutableMapOf<String, Int>()

    fun reset() {
        startTime = Timer.getFPGATimestamp()
    }

    /** Log the period and the time of the function.  */
    fun record(loopName: String): Pair<Double, Double> {
        val loopIndex: Int = loopsAndIndex.getOrPut(loopName) { loopsAndIndex.size }
        val now = Timer.getFPGATimestamp()
        val prevTime = prevTimes.put(loopName, now) ?: now //put returns previous value
        val period = now - prevTime
        val sinceReset = now - startTime

        Logger.recordOutput("LoopLogger/Period/$loopIndex $loopName", period)
        Logger.recordOutput("LoopLogger/SinceReset/$loopIndex $loopName", sinceReset)
        return Pair(period, sinceReset)
    }
}