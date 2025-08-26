package frc.team2471.off2025

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.StrictFollower
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.LoopLogger
import frc.team2471.off2025.util.setMagnetSensorOffset
import frc.team2471.off2025.util.units.asDegrees
import frc.team2471.off2025.util.units.asFeetPerSecondPerSecond
import frc.team2471.off2025.util.units.asInches
import frc.team2471.off2025.util.units.asRotations
import frc.team2471.off2025.util.units.cos
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.inches
import frc.team2471.off2025.util.units.feet
import frc.team2471.off2025.util.units.rotations
import frc.team2471.off2025.util.units.sin
import frc.team2471.off2025.util.units.wrap
import org.littletonrobotics.junction.Logger

object Armavator: SubsystemBase() {
    private val table = NetworkTableInstance.getDefault().getTable("Armavator")

    private val armEncoderOffsetEntry = table.getEntry("Arm Encoder Offset")
    private val pivotEncoderOffsetEntry = table.getEntry("Pivot Encoder Offset")

    val elevatorMotor0 = TalonFX(Falcons.ELEVATOR_0, CANivores.ELEVATOR_CAN)
    val elevatorMotor1 = TalonFX(Falcons.ELEVATOR_1, CANivores.ELEVATOR_CAN)
    val elevatorMotor2 = TalonFX(Falcons.ELEVATOR_2, CANivores.ELEVATOR_CAN)
    val elevatorMotor3 = TalonFX(Falcons.ELEVATOR_3, CANivores.ELEVATOR_CAN)

    val armMotor0 = TalonFX(Falcons.ARM_MOTOR_0, CANivores.ELEVATOR_CAN)
    val armMotor1 = TalonFX(Falcons.ARM_MOTOR_1, CANivores.ELEVATOR_CAN)

    val armCanCoder = CANcoder(CANCoders.ARM, CANivores.ELEVATOR_CAN)

    val pivotMotor = TalonFX(Falcons.PIVOT_MOTOR, CANivores.ELEVATOR_CAN)

    const val ELEVATOR_REVOLUTIONS_PER_INCH = 1.0/1.6
    const val MIN_HEIGHT_INCHES = 0.0
    const val MAX_HEIGHT_INCHES = 58.0

    const val ARM_GEAR_RATIO = 60.0
    const val MIN_ARM_ANGLE_DEGREES = -113.0
    const val MAX_ARM_ANGLE_DEGREES = 113.0

    const val PIVOT_GEAR_RATIO = 360.0 / (33.0 + 1.0/3.0)

    inline val currentHeight: Distance
        get() = (elevatorMotor0.position.valueAsDouble / ELEVATOR_REVOLUTIONS_PER_INCH).inches

    inline val currentArmAngle: Angle
        get() = armMotor0.position.valueAsDouble.rotations

    inline val currentPivotAngle: Angle
        get() = (pivotMotor.position.valueAsDouble / PIVOT_GEAR_RATIO).rotations

    val defaultPivotEncoderOffset =
        if (Robot.isCompBot) -1.841 else 0.0
    val defaultArmEncoderOffset =
        if (Robot.isCompBot) 195.8 else -129.9

    var pivotEncoderOffset: Double = pivotEncoderOffsetEntry.getDouble(defaultPivotEncoderOffset)
    var armEncoderOffset: Double = armEncoderOffsetEntry.getDouble(defaultArmEncoderOffset)

    val elevatorFeedforward: Double
        get() = if (currentHeight < 20.0.inches) {
            -0.04
        } else {
            0.04
        }

    const val PIVOT_STATIC_FEED_FORWARD = 0.015
    val pivotFeedForward: Double get() = (0.055 * currentPivotAngle.wrap().sin()) * currentArmAngle.sin()


    val armFeedForward: Double get() = 0.04 * (1.0 + (elevatorMotor0.acceleration.valueAsDouble * ELEVATOR_REVOLUTIONS_PER_INCH / 32.0.feet.asInches)) * -(currentArmAngle + (0.0.degrees * (currentPivotAngle + 90.0.degrees).sin())).sin() +
            0.04 * (Drive.acceleration.rotateBy(-Drive.heading).x.asFeetPerSecondPerSecond / 32.0) * currentArmAngle.cos()


    var heightSetpoint: Distance = 0.0.inches
        set(value) {
            val safeValue = MathUtil.clamp(value.asInches, MIN_HEIGHT_INCHES, MAX_HEIGHT_INCHES)
            elevatorMotor0.setControl(MotionMagicDutyCycle(safeValue * ELEVATOR_REVOLUTIONS_PER_INCH).withFeedForward(elevatorFeedforward))
//            println("elevator position setpoint: $value")
            field = value
        }

    var armAngleSetpoint: Angle = 0.0.degrees
        set(value) {
            val safeValue = MathUtil.clamp(value.asDegrees, MIN_ARM_ANGLE_DEGREES, MAX_ARM_ANGLE_DEGREES)
            armMotor0.setControl(MotionMagicDutyCycle(safeValue.degrees.asRotations).withFeedForward(armFeedForward))
//            println("arm angle setpoint: ${value.asDegrees}")
            field = safeValue.degrees
        }

    var pivotAngleSetpoint: Angle = 0.0.degrees
        set(value) {
//            pivotMotor.setControl(MotionMagicDutyCycle(value.asRotations * PIVOT_GEAR_RATIO).withFeedForward(pivotFeedForward + PIVOT_STATIC_FEED_FORWARD))
//            println("pivot angle setpoint: ${value.asDegrees}")
            field = value
        }
    val isArmFlipped: Boolean
        get() = currentArmAngle < 0.0.degrees
    val isPivotFlipped: Boolean
        get() = 0.0.degrees.wrap() < 0.0.degrees
    val reverseSpitDirection: Boolean
        get() = isArmFlipped != isPivotFlipped


    init {
        println("inside Armavator init")

        if (!armEncoderOffsetEntry.exists()) {
            armEncoderOffsetEntry.setDouble(defaultArmEncoderOffset)
            println("Shoulder didn't exist!!!!!!!!!!!!!")
        }
        if (!pivotEncoderOffsetEntry.exists()) {
            pivotEncoderOffsetEntry.setDouble(defaultPivotEncoderOffset)
            println("Elbow didn't exist!!!!!!!!!!!!!!!")
        }

        armCanCoder.configurator.apply(CANcoderConfiguration().apply {
            MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
            MagnetSensor.MagnetOffset = armEncoderOffset.degrees.asRotations
        })


        val elevatorMotorConfigs = TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimit = 30.0
                SupplyCurrentLimitEnable = true
            }
            MotorOutput.apply {
                Inverted = InvertedValue.Clockwise_Positive
                NeutralMode = NeutralModeValue.Coast
            }
            Slot0.apply {
                kP = 0.5
//                kG = 0.06
//                GravityType = GravityTypeValue.Elevator_Static
            }
            MotionMagic.apply {
                MotionMagicAcceleration = 25.0 * ELEVATOR_REVOLUTIONS_PER_INCH
                MotionMagicCruiseVelocity = 35.0 * ELEVATOR_REVOLUTIONS_PER_INCH
            }
        }

        elevatorMotor0.configurator.apply(elevatorMotorConfigs)
        elevatorMotor1.configurator.apply(elevatorMotorConfigs)
        elevatorMotor2.configurator.apply(elevatorMotorConfigs)
        elevatorMotor3.configurator.apply(elevatorMotorConfigs)


        elevatorMotor1.setControl(Follower(elevatorMotor0.deviceID, true))
        elevatorMotor2.setControl(Follower(elevatorMotor0.deviceID, false))
        elevatorMotor3.setControl(Follower(elevatorMotor0.deviceID, false))

        val armMotorConfigs = TalonFXConfiguration().apply { // TODO: tune
            CurrentLimits.apply {
                SupplyCurrentLimit = 30.0
                SupplyCurrentLimitEnable = true
            }
            MotorOutput.apply {
                Inverted = InvertedValue.CounterClockwise_Positive
                NeutralMode = NeutralModeValue.Brake
            }
            Slot0.apply {
                kP = 8.3333
                kD = 0.69
            }
            MotionMagic.apply {
                MotionMagicAcceleration = 7.0 * 360.0 * ARM_GEAR_RATIO
                MotionMagicCruiseVelocity = 360.0 * ARM_GEAR_RATIO
            }
            Feedback.apply {
                FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
                FeedbackRemoteSensorID = CANCoders.ARM
                RotorToSensorRatio = ARM_GEAR_RATIO
            }
        }

        armMotor0.configurator.apply(armMotorConfigs)
        armMotor1.configurator.apply(armMotorConfigs)

        armMotor1.setControl(StrictFollower(armMotor0.deviceID))
    }

    override fun periodic() {
        LoopLogger.record("b4 Armavator pirdc")

        Logger.recordOutput("Armavator/currentHeight", currentHeight.asInches)
        Logger.recordOutput("Armavator/currentArmAngle", currentArmAngle.asDegrees)
        Logger.recordOutput("Armavator/currentArmEncoderAngle", armCanCoder.position.valueAsDouble.rotations.asDegrees)
        Logger.recordOutput("Armavator/currentPivotAngle", currentPivotAngle.asDegrees)

        Logger.recordOutput("Armavator/heightSetpoint", heightSetpoint.asInches)
        Logger.recordOutput("Armavator/armAngleSetpoint", armAngleSetpoint.asDegrees)
        Logger.recordOutput("Armavator/pivotAngleSetpoint", pivotAngleSetpoint.asDegrees)

        Logger.recordOutput("Armavator/elevatorCurrent", elevatorMotor0.supplyCurrent.valueAsDouble)
        Logger.recordOutput("Armavator/armCurrent", armMotor0.supplyCurrent.valueAsDouble)
        Logger.recordOutput("Armavator/pivotCurrent", pivotMotor.supplyCurrent.valueAsDouble)

        Logger.recordOutput("Armavator/elevatorVelocity", elevatorMotor0.velocity.valueAsDouble)
        Logger.recordOutput("Armavator/armVelocity", armMotor0.velocity.valueAsDouble)
        Logger.recordOutput("Armavator/pivotVelocity", pivotMotor.velocity.valueAsDouble)

        Logger.recordOutput("Armavator/elevatorFeedforward", elevatorFeedforward)
        Logger.recordOutput("Armavator/armFeedforward", armFeedForward)
        Logger.recordOutput("Armavator/pivotFeedforward", pivotFeedForward)

        heightSetpoint = heightSetpoint
        armAngleSetpoint = armAngleSetpoint
        pivotAngleSetpoint = pivotAngleSetpoint

        LoopLogger.record("Armavator pirdc")
    }

    /**
    percent is a double between -1 and 1
     **/
    fun setElevatorPercentOut(percent: Double) {
        elevatorMotor0.setControl(DutyCycleOut(percent))
        println("elevator percentage: $percent")
    }

    fun goToPose(pose: Pose) {
        heightSetpoint = pose.elevatorHeight
        armAngleSetpoint = pose.armAngle
        pivotAngleSetpoint = pose.pivotAngle
    }
}