package frc.team2471.off2025

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc2025.CANivores
import org.team2471.frc2025.Falcons

object Intake: SubsystemBase("Intake") {
    var intakeState: IntakeState = IntakeState.HOLDING
    val frontMotor = TalonFX(Falcons.FRONT_INTAKE, CANivores.ELEVATOR_CAN)
    val sideMotor = TalonFX(Falcons.SIDE_MOTOR, CANivores.ELEVATOR_CAN)

    val INTAKE_POWER = -0.6
    val SIDE_MOVE_POWER: Double = if (Robot.isCompBot) -0.1 else 0.1
    val ALGAE_INTAKE_POWER = 0.6
    val ALGAE_INTAKE_POWER_AUTO = 1.0
    val SIDE_SPIT_POWER = 0.8
    init {
        frontMotor.configurator.apply (
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimit = 30.0
                    SupplyCurrentLowerLimit = 5.0
                    SupplyCurrentLowerTime = 0.5
                    SupplyCurrentLimitEnable = true
                }
            }
        )
        sideMotor.configurator.apply(
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimit = 40.0
                    SupplyCurrentLowerLimit = 20.0
                    SupplyCurrentLowerTime = 1.0
                    SupplyCurrentLimitEnable = true
                }
                MotorOutput.apply {
                    NeutralMode = NeutralModeValue.Brake
                    Inverted = InvertedValue.Clockwise_Positive
                }
            }
        )
    }

    override fun periodic() {
        when (intakeState) {
            IntakeState.INTAKING -> {
                frontMotor.setControl(DutyCycleOut(INTAKE_POWER))
                sideMotor.setControl(DutyCycleOut(0.0))
            }
            IntakeState.REVERSING -> {
                frontMotor.setControl(DutyCycleOut(ALGAE_INTAKE_POWER_AUTO))
                sideMotor.setControl(DutyCycleOut(0.0))
            }
            IntakeState.HOLDING -> {
                frontMotor.setControl(DutyCycleOut(0.2))
                sideMotor.setControl(DutyCycleOut(0.0))
            }
            IntakeState.SCORING -> {
                frontMotor.setControl(DutyCycleOut(0.1))
                sideSplit()
            }
        }
    }
    private fun sideSplit() {
        if (Armavator.reverseSpitDirection) {
            sideMotor.setControl(DutyCycleOut(-SIDE_SPIT_POWER))
        } else {
            sideMotor.setControl(DutyCycleOut(SIDE_SPIT_POWER))
        }
    }
}

enum class IntakeState {
    INTAKING,
    REVERSING,
    HOLDING,
    SCORING
}