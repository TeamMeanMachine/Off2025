package frc.team2471.off2025

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.control.commands.sequenceCommand
import frc.team2471.off2025.util.control.commands.use
import frc.team2471.off2025.util.control.commands.waitUntilCommand
import frc.team2471.off2025.util.ctre.applyConfiguration
import frc.team2471.off2025.util.ctre.currentLimits
import frc.team2471.off2025.util.units.asDegrees
import frc.team2471.off2025.util.units.rotations
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

object Climb: SubsystemBase("Climb") {
    val climberMotor = TalonFX(Falcons.CLIMBER_MOTOR)
    val switch = DigitalInput(DigitalSensors.CLIMBER)

    const val DEPLOY_ROTATIONS = 0.5
    const val FEEDBACK_COEFFICIENT = 1.0/45.0

    val relayOn: Boolean get() = !switch.get()

    var hasDeployed = false

    val currentMotorPos: Double
        get() = climberMotor.position.valueAsDouble * FEEDBACK_COEFFICIENT

    var motorPercentOutput = 0.0
        set(value) {
            val safeValue = if (relayOn) 0.0 else value
            climberMotor.setControl(DutyCycleOut(safeValue))
            field = safeValue
        }


    init {
        println("Climb init")

        // configure climber motor.
        // 20A continuous limit, 30A peak limit, 1 second peak duration
        // coast mode, no invert
        // gear ratio is 45:1

        climberMotor.applyConfiguration {
            currentLimits(20.0, 30.0, 1.0)
        }
    }


    // Logging periodic and set motor power to 0.0 if the relay is active.
    override fun periodic() {
        Logger.recordOutput("Climb/relayOn", relayOn)
        Logger.recordOutput("Climb/motorPower", climberMotor.dutyCycle.valueAsDouble)
        Logger.recordOutput("Climb/motorPositionDeg", climberMotor.position.valueAsDouble.rotations.asDegrees)
        if (relayOn) {
            motorPercentOutput = 0.0
        }
    }


    // function that can "deploy" the climber by doing a 180-degree turn (or more) forward. A PID or bang-bang controller should work
    fun deploy(): Command {
        var startingAngle = currentMotorPos
        return sequenceCommand(
            runOnce {
                hasDeployed = true
                startingAngle = currentMotorPos
                motorPercentOutput = 1.0
            },
            waitUntilCommand { abs(currentMotorPos - startingAngle) > DEPLOY_ROTATIONS },
            runOnce {
                motorPercentOutput = 0.0
            }
        )
    }
}