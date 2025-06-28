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
    fun record(loopName: String) {
        val now = Timer.getFPGATimestamp()
        val prevTime = prevTimes[loopName] ?: now
        Logger.recordOutput("LoopLogger/$loopName/Period", (now - prevTime))
        Logger.recordOutput("LoopLogger/$loopName/SinceReset", (now - startTime))
        prevTimes[loopName] = now
    }
}