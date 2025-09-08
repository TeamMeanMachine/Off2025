package frc.team2471.off2025

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Intake: SubsystemBase("Intake") {
    var intakeState: IntakeState = IntakeState.HOLDING
    val frontMotor = TalonFX(Falcons.FRONT_INTAKE, CANivores.ELEVATOR_CAN)
    val sideMotor = TalonFX(Falcons.SIDE_MOTOR, CANivores.ELEVATOR_CAN)

    val canRangeLeftDistFilter = LinearFilter.movingAverage(5)
    val canRangeRightDistFilter = LinearFilter.movingAverage(5)
    var canRangeLeftDist = 0.0
    var canRangeRightDist = 0.0
    val cargoDetectedLeft get() = if (canRangeLeftDist != 0.0) canRangeLeftDist < if (Robot.isCompBot) 0.09 else 0.08 else false
    val cargoDetectedRight get() = if (canRangeRightDist != 0.0) canRangeRightDist < if (Robot.isCompBot) 0.09 else 0.08 else false
    val canRangeLeft = CANrange(CANSensors.RANGE_INTAKE_LEFT, CANivores.ELEVATOR_CAN)
    val canRangeRight = CANrange(CANSensors.RANGE_INTAKE_RIGHT, CANivores.ELEVATOR_CAN)

    var hasCargo: Boolean = false
    var scoreAlgae = false

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
                    Inverted = InvertedValue.CounterClockwise_Positive
                }
            }
        )
    }

    override fun periodic() {
        canRangeLeftDist = canRangeLeftDistFilter.calculate(canRangeLeft.distance.valueAsDouble)
        canRangeRightDist = canRangeRightDistFilter.calculate(canRangeRight.distance.valueAsDouble)
        when (intakeState) {
            IntakeState.INTAKING -> {
                frontMotor.setControl(DutyCycleOut(INTAKE_POWER))
                sideMotor.setControl(DutyCycleOut(0.0))
                centeringLogic(Robot.isAutonomous)
            }
            IntakeState.REVERSING -> {
                frontMotor.setControl(DutyCycleOut(ALGAE_INTAKE_POWER_AUTO))
                sideMotor.setControl(DutyCycleOut(0.0))
                centeringLogic()
            }
            IntakeState.HOLDING -> {
                frontMotor.setControl(DutyCycleOut(0.2))
                sideMotor.setControl(DutyCycleOut(0.0))
                centeringLogic()
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
    fun centeringLogic(runSideMotorWhenUnseen: Boolean = false) {
        if (cargoDetectedRight) {
            sideMotor.setControl(DutyCycleOut(-SIDE_MOVE_POWER))
            hasCargo = true
        } else if (cargoDetectedLeft) {
            sideMotor.setControl(DutyCycleOut(SIDE_MOVE_POWER))
            hasCargo = true
        } else {
            sideMotor.setControl(DutyCycleOut(if (runSideMotorWhenUnseen) -SIDE_MOVE_POWER else 0.0))
        }
    }
    fun score() {
        if (scoreAlgae) {
            intakeState = IntakeState.INTAKING
        } else {
            intakeState = IntakeState.SCORING
        }
    }
}

enum class IntakeState {
    INTAKING,
    REVERSING,
    HOLDING,
    SCORING
}