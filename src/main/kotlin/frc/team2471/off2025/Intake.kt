package frc.team2471.off2025

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.ctre.applyConfiguration
import frc.team2471.off2025.util.ctre.brakeMode
import frc.team2471.off2025.util.ctre.currentLimits
import frc.team2471.off2025.util.ctre.inverted

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
        frontMotor.applyConfiguration {
            currentLimits(30.0, 5.0, 0.5)
        }
        sideMotor.applyConfiguration {
            currentLimits(20.0, 40.0, 1.0)
            brakeMode()
            inverted(false)
        }
    }

    override fun periodic() {
        canRangeLeftDist = canRangeLeftDistFilter.calculate(canRangeLeft.distance.valueAsDouble)
        canRangeRightDist = canRangeRightDistFilter.calculate(canRangeRight.distance.valueAsDouble)
        when (intakeState) {
            IntakeState.INTAKING -> {
                frontMotor.setControl(VoltageOut(INTAKE_POWER * 12.0))
                sideMotor.setControl(DutyCycleOut(0.0))
                centeringLogic(Robot.isAutonomous)
            }
            IntakeState.REVERSING -> {
                frontMotor.setControl(VoltageOut(ALGAE_INTAKE_POWER_AUTO * 12.0))
                sideMotor.setControl(DutyCycleOut(0.0))
                centeringLogic()
            }
            IntakeState.HOLDING -> {
                frontMotor.setControl(DutyCycleOut(0.2))
                sideMotor.setControl(DutyCycleOut(0.0))
                centeringLogic()
            }
            IntakeState.SCORING -> {
                frontMotor.setControl(VoltageOut(0.1 * 12.0))
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