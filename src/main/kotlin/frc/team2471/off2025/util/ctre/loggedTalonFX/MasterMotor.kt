package frc.team2471.off2025.util.ctre.loggedTalonFX

import frc.team2471.off2025.util.isSim

object MasterMotor {
    private val motors = mutableListOf<LoggedMotor>()

    fun simPeriodic() {
        if (isSim) {
            motors.forEach {
                it.simPeriodic()
            }
        }
    }

    fun addMotor(motor: LoggedMotor) = motors.add(motor)
}