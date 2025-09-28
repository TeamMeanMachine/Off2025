package frc.team2471.off2025

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

object Climb: SubsystemBase("Climb") {
    val climberMotor = TalonFX(Falcons.CLIMBER_MOTOR)
    val switch = DigitalInput(DigitalSensors.CLIMBER)

    var climbPowerSetpoint: Double = 0.0
        set(value) {
            if (relayOn) {
                field = 0.0 // Relay is on, stop moving
            } else {
                field = value.coerceIn(0.0, 1.0) // Motor should never go backwards.
            }
            climberMotor.setControl(DutyCycleOut(field))
        }

    val relayOn: Boolean get() = !switch.get()


    init {
        println("Climb init")
        // configure climber motor.
        // 20A continuous limit, 30A peak limit, 1 second peak duration
        // coast mode, no invert
        // gear ratio is 45:1
    }


    override fun periodic() {
        if (relayOn) {
            climbPowerSetpoint = 0.0
        }
        Logger.recordOutput("Climb/relayOn", relayOn)
        Logger.recordOutput("Climb/powerSetpoint", climbPowerSetpoint)
    }


    // needs function that can "deploy" the climber by doing a 180 degree turn (or more) forward. A PID or bang-bang controller should work
}