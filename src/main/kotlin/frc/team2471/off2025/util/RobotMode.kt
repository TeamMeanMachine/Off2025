package frc.team2471.off2025.util

import edu.wpi.first.hal.HALUtil
import edu.wpi.first.wpilibj.RuntimeType


val doReplay: Boolean = false
val robotMode: RobotMode = when (RuntimeType.getValue(HALUtil.getHALRuntimeType())) {
    RuntimeType.kRoboRIO2, RuntimeType.kRoboRIO -> RobotMode.REAL
    RuntimeType.kSimulation -> if (doReplay) RobotMode.REPLAY else RobotMode.SIM
    else -> RobotMode.REAL
}.also { println("robotMode = $it") }

val isReal = robotMode == RobotMode.REAL
val isSim = !isReal
val isReplay = robotMode == RobotMode.REPLAY


enum class RobotMode {
    REAL,
    SIM,
    REPLAY
}