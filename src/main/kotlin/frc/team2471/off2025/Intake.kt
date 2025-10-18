package frc.team2471.off2025

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.units.asFahrenheit
import org.littletonrobotics.junction.Logger

object Intake: SubsystemBase("Intake") {
    var intakeState: IntakeState = IntakeState.HOLDING
    val frontMotor = TalonFX(Falcons.FRONT_INTAKE, CANivores.ELEVATOR_CAN)
    val sideMotor = TalonFX(Falcons.SIDE_MOTOR, CANivores.ELEVATOR_CAN)

    val canRangeLeftDistFilter = LinearFilter.movingAverage(5)
    val canRangeRightDistFilter = LinearFilter.movingAverage(5)
    var canRangeLeftDist = 0.0
    var canRangeRightDist = 0.0
    val cargoDetectedLeft get() = if (canRangeLeftDist != 0.0) canRangeLeftDist < if (Robot.isCompBot) 0.08 else 0.08 else false
    val cargoDetectedRight get() = if (canRangeRightDist != 0.0) canRangeRightDist < if (Robot.isCompBot) 0.08 else 0.08 else false
    val canRangeLeft = CANrange(CANSensors.RANGE_INTAKE_LEFT, CANivores.ELEVATOR_CAN)
    val canRangeRight = CANrange(CANSensors.RANGE_INTAKE_RIGHT, CANivores.ELEVATOR_CAN)

    var hasCargo: Boolean = false
    var scoreAlgae = false

    var afterDisabled = false
    private var prevIntakeState = intakeState

    const val INTAKE_POWER = -0.6
    const val AUTO_INTAKE_POWER = -0.9
    val sideMovePower: Double = if (Robot.isCompBot) -0.1 else 0.1
    const val ALGAE_INTAKE_POWER = 1.0
    const val ALGAE_GROUND_INTAKE_POWER = 0.6
    const val SIDE_SPIT_POWER = 0.8

    val scoredTimer = Timer()
    val timeSinceLastScore get() = scoredTimer.get()

    init {
        frontMotor.applyConfiguration {
            currentLimits(5.0, 30.0, 0.5)
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
                frontMotor.setControl(VoltageOut((if (Robot.isAutonomous) AUTO_INTAKE_POWER else INTAKE_POWER) * 12.0))
                centeringLogic(Robot.isAutonomous)
            }
            IntakeState.ALGAE_GROUND -> {
                frontMotor.setControl(VoltageOut(ALGAE_GROUND_INTAKE_POWER * 12.0))
                centeringLogic()
            }
            IntakeState.ALGAE_DESCORE -> {
                frontMotor.setControl(VoltageOut(ALGAE_INTAKE_POWER * 12.0))
                centeringLogic()
            }
            IntakeState.HOLDING -> {
                if (afterDisabled) {
                    frontMotor.setControl(DutyCycleOut(0.0))
                } else {
                    frontMotor.setControl(DutyCycleOut(ALGAE_GROUND_INTAKE_POWER))
                }
                centeringLogic()
            }
            IntakeState.SCORING -> {
                if (scoreAlgae) {
                    frontMotor.setControl(DutyCycleOut(-1.0))
                    centeringLogic()
                } else {
                    frontMotor.setControl(VoltageOut(0.1 * 12.0))
                    sideSplit()
                }
            }
        }
        if (afterDisabled) {
            if (prevIntakeState != intakeState) {
                afterDisabled = false
            }
        }
        Logger.recordOutput("Intake/FrontTemp", frontMotor.deviceTemp.value.asFahrenheit)
        Logger.recordOutput("Intake/IntakeState", intakeState.name)
        Logger.recordOutput("Intake/FrontOutputV", frontMotor.motorVoltage.valueAsDouble)
        Logger.recordOutput("Intake/SideOutputV", sideMotor.motorVoltage.valueAsDouble)
        Logger.recordOutput("Intake/hasCargo", hasCargo)
        Logger.recordOutput("Intake/timeSinceLastScore", timeSinceLastScore)
        Logger.recordOutput("Intake/cargoDetectedRight", cargoDetectedRight)
        Logger.recordOutput("Intake/cargoDetectedLeft", cargoDetectedLeft)
        Logger.recordOutput("Intake/rightDistance", canRangeRightDist)
        Logger.recordOutput("Intake/leftDistance", canRangeLeftDist)
    }

    private fun sideSplit() {
        if (Armavator.reverseSpitDirection) {
            sideMotor.setControl(DutyCycleOut(-SIDE_SPIT_POWER))
        } else {
            sideMotor.setControl(DutyCycleOut(SIDE_SPIT_POWER))
        }
        hasCargo = false
    }
    fun centeringLogic(runSideMotorWhenUnseen: Boolean = false) {
        if (cargoDetectedRight) {
            sideMotor.setControl(DutyCycleOut(-sideMovePower))
            hasCargo = true
        } else if (cargoDetectedLeft) {
            sideMotor.setControl(DutyCycleOut(sideMovePower))
            hasCargo = true
        } else {
            sideMotor.setControl(DutyCycleOut(if (runSideMotorWhenUnseen) -sideMovePower else 0.0))
        }
    }
    fun score() {
        intakeState = IntakeState.SCORING
    }
}

enum class IntakeState {
    INTAKING,
    ALGAE_GROUND,
    HOLDING,
    SCORING,
    ALGAE_DESCORE

}