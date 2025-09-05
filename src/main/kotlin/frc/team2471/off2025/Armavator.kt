package frc.team2471.off2025

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.CANdi
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.off2025.util.control.LoopLogger
import frc.team2471.off2025.util.ctre.addFollower
import frc.team2471.off2025.util.ctre.brakeMode
import frc.team2471.off2025.util.ctre.currentLimits
import frc.team2471.off2025.util.ctre.d
import frc.team2471.off2025.util.ctre.p
import frc.team2471.off2025.util.ctre.remoteCANCoder
import frc.team2471.off2025.util.ctre.s
import frc.team2471.off2025.util.units.asDegrees
import frc.team2471.off2025.util.units.asFeetPerSecondPerSecond
import frc.team2471.off2025.util.units.asInches
import frc.team2471.off2025.util.units.asRadians
import frc.team2471.off2025.util.units.asRotations
import frc.team2471.off2025.util.units.cos
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.inches
import frc.team2471.off2025.util.units.feet
import frc.team2471.off2025.util.units.rotations
import frc.team2471.off2025.util.units.sin
import frc.team2471.off2025.util.units.unWrap
import frc.team2471.off2025.util.units.wrap
import motion_profiling.MotionCurve
import org.littletonrobotics.junction.Logger
import kotlin.math.IEEErem
import kotlin.math.abs
import kotlin.math.absoluteValue

object Armavator: SubsystemBase() {
    private val table = NetworkTableInstance.getDefault().getTable("Armavator")

    private val armEncoderOffsetEntry = table.getEntry("Arm Encoder Offset")
    private val elevatorEncoderOffsetEntry = table.getEntry("Elevator Encoder Offset")

    private val pivotEncoderOffsetEntry = table.getEntry("Pivot Encoder Offset")

    val elevatorMotor = TalonFX(Falcons.ELEVATOR_0, CANivores.ELEVATOR_CAN)

    val armMotor = TalonFX(Falcons.ARM_MOTOR_0, CANivores.ELEVATOR_CAN)

    val candi = CANdi(CANSensors.CANDI, CANivores.ELEVATOR_CAN)

    val pivotMotor = TalonFX(Falcons.PIVOT_MOTOR, CANivores.ELEVATOR_CAN)

    inline val rawPivotAngle: Angle
        get() = -candi.pwM1Position.valueAsDouble.IEEErem(1.0).rotations.wrap()

    inline val candiAngle: Angle
        get() = (rawPivotAngle - pivotEncoderOffset.degrees).wrap()

    inline val lampreyAlignmentOffset: Angle
        get() = -abs((kotlin.math.sin(candiAngle.asRadians) * 12.0)).degrees

    inline val pivotEncoderAngle: Angle
        get() = (candiAngle - lampreyAlignmentOffset).wrap()

    inline val rawElevatorEncoderValue: Double
        get() = (elevatorCANcoder.position.valueAsDouble)

    inline val elevatorEncoderHeight: Distance
        get() = elevatorEncoderCurve.getValue(rawElevatorEncoderValue).inches

    inline val pivotMotorAngle: Angle
        get() = (pivotMotor.position.valueAsDouble.rotations/PIVOT_GEAR_RATIO)

    val armCanCoder = CANcoder(CANCoders.ARM, CANivores.ELEVATOR_CAN)

    val elevatorCANcoder = CANcoder(CANCoders.ELEVATOR)
    val elevatorEncoderCurve = MotionCurve()

    const val ELEVATOR_REVOLUTIONS_PER_INCH = 1.0/1.6
    const val MIN_HEIGHT_INCHES = 0.0
    const val MAX_HEIGHT_INCHES = 58.0

    const val ARM_GEAR_RATIO = 60.0
    const val MIN_ARM_ANGLE_DEGREES = -113.0
    const val MAX_ARM_ANGLE_DEGREES = 113.0

    const val PIVOT_GEAR_RATIO = 33.0 + 1.0/3.0
    const val DEFAULT_PIVOT_ACCEL = 25.0 * 360.0
    const val DEFAULT_PIVOT_CRUISING_VEL = 2.0 * 360.0


    inline val currentHeight: Distance
        get() = (elevatorMotor.position.valueAsDouble).inches

    inline val currentArmAngle: Angle
        get() = armMotor.position.valueAsDouble.rotations


    val defaultPivotEncoderOffset =
        if (Robot.isCompBot) -1.841 else 0.0
    val defaultArmEncoderOffset =
        if (Robot.isCompBot) 195.8 else -129.9
    val defaultElevatorEncoderOffset =
        if (Robot.isCompBot) 0.205078125 else 0.205078125

    var pivotEncoderOffset: Double = pivotEncoderOffsetEntry.getDouble(defaultPivotEncoderOffset)
    var armEncoderOffset: Double = armEncoderOffsetEntry.getDouble(defaultArmEncoderOffset)
    var elevatorEncoderOffset: Double = elevatorEncoderOffsetEntry.getDouble(defaultElevatorEncoderOffset)


    val elevatorFeedforward: Double
        get() = if (currentHeight < 20.0.inches) {
            -0.04
        } else {
            0.04
        }

    const val PIVOT_STATIC_FEED_FORWARD = 0.07912
    val pivotFeedForward: Double get() = (0.055 * pivotMotorAngle.wrap().sin()) * currentArmAngle.sin()


    val armFeedForward: Double get() = 0.04 * (1.0 + (elevatorMotor.acceleration.valueAsDouble / 32.0.feet.asInches)) * -(currentArmAngle + (0.0.degrees * (pivotMotorAngle + 90.0.degrees).sin())).sin() +
            0.04 * (Drive.acceleration.rotateBy(-Drive.heading).x.asFeetPerSecondPerSecond / 32.0) * currentArmAngle.cos()


    var heightSetpoint: Distance = 0.0.inches
        set(value) {
            field = MathUtil.clamp(value.asInches, MIN_HEIGHT_INCHES, MAX_HEIGHT_INCHES).inches
            elevatorMotor.setControl(MotionMagicDutyCycle(field.asInches).withFeedForward(elevatorFeedforward))
//            println("elevator position setpoint: $value")
        }

    var armAngleSetpoint: Angle = 0.0.degrees
        set(value) {
            field = MathUtil.clamp(value.asDegrees, MIN_ARM_ANGLE_DEGREES, MAX_ARM_ANGLE_DEGREES).degrees
            armMotor.setControl(MotionMagicDutyCycle(field.asRotations).withFeedForward(armFeedForward))
//            println("arm angle setpoint: ${value.asDegrees}")
        }

    var pivotAngleSetpoint: Angle = 0.0.degrees
        set(value) {
            field = value.unWrap(pivotMotorAngle)
            pivotMotor.setControl(MotionMagicVoltage(field.asRotations * PIVOT_GEAR_RATIO).withFeedForward(pivotFeedForward * 12.0))
//            println("pivot angle setpoint: ${field.asDegrees}")
        }
    val isArmFlipped: Boolean
        get() = currentArmAngle < 0.0.degrees
    val isPivotFlipped: Boolean
        get() = pivotMotorAngle.wrap() > 0.0.degrees
    val reverseSpitDirection: Boolean
        get() = isArmFlipped != isPivotFlipped

    var periodicFeedForward = true


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

        elevatorCANcoder.configurator.apply(CANcoderConfiguration().apply {
            MagnetSensor.MagnetOffset = elevatorEncoderOffset
            MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
        })

        // found that it is about 5.8 inches per rotation
        elevatorEncoderCurve.storeValue(0.0, 0.0)
        elevatorEncoderCurve.storeValue(0.305, 1.53)
        elevatorEncoderCurve.storeValue(2.544, 14.85)
        elevatorEncoderCurve.storeValue(3.41, 19.944)
        elevatorEncoderCurve.storeValue(3.87, 22.63)
        elevatorEncoderCurve.storeValue(4.728, 27.60)
        elevatorEncoderCurve.storeValue(5.643, 32.87)
        elevatorEncoderCurve.storeValue(9.99, 62.31)
        elevatorEncoderCurve.storeValue(8.175, 49.233)
        elevatorEncoderCurve.storeValue(6.237, 38.142)

        pivotMotor.setPosition(pivotEncoderAngle * PIVOT_GEAR_RATIO)


        pivotMotor.configurator.apply(TalonFXConfiguration().apply {
            p(5.0)
            d(0.2)
            s(PIVOT_STATIC_FEED_FORWARD, StaticFeedforwardSignValue.UseClosedLoopSign)

            currentLimits(20.0, 30.0, 1.0)
            brakeMode()

            MotionMagic.apply {
                MotionMagicAcceleration = DEFAULT_PIVOT_ACCEL
                MotionMagicCruiseVelocity = DEFAULT_PIVOT_CRUISING_VEL
            }
        })


        elevatorMotor.configurator.apply(TalonFXConfiguration().apply {
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
                MotionMagicAcceleration = 150.0
                MotionMagicCruiseVelocity = 35.0
            }
            remoteCANCoder(elevatorCANcoder.deviceID, 5.8 / ELEVATOR_REVOLUTIONS_PER_INCH, 1.0/5.8)
        })

        elevatorMotor.addFollower(Falcons.ELEVATOR_1, true)
        elevatorMotor.addFollower(Falcons.ELEVATOR_2, false)
        elevatorMotor.addFollower(Falcons.ELEVATOR_2, false)



        armMotor.configurator.apply(TalonFXConfiguration().apply {
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
                MotionMagicAcceleration = 7.0 * 360.0 * ARM_GEAR_RATIO / 40.0
                MotionMagicCruiseVelocity = 360.0 * ARM_GEAR_RATIO / 40.0
            }
            Feedback.apply {
                FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
                FeedbackRemoteSensorID = CANCoders.ARM
                RotorToSensorRatio = ARM_GEAR_RATIO
            }
        })

        armMotor.addFollower(Falcons.ARM_MOTOR_1)
    }

    override fun periodic() {
        LoopLogger.record("b4 Armavator pirdc")

        Logger.recordOutput("Armavator/currentHeight", currentHeight.asInches)
        Logger.recordOutput("Armavator/currentArmAngle", currentArmAngle.asDegrees)
        Logger.recordOutput("Armavator/currentArmEncoderAngle", armCanCoder.position.valueAsDouble.rotations.asDegrees)
        Logger.recordOutput("Armavator/pivotEncoderAngle", pivotEncoderAngle.asDegrees)
        Logger.recordOutput("Armavator/pivotMotorAngle", pivotMotorAngle.asDegrees)

        Logger.recordOutput("Armavator/heightSetpoint", heightSetpoint.asInches)
        Logger.recordOutput("Armavator/armAngleSetpoint", armAngleSetpoint.asDegrees)
        Logger.recordOutput("Armavator/pivotAngleSetpoint", pivotAngleSetpoint.asDegrees)

        Logger.recordOutput("Armavator/elevatorCurrent", elevatorMotor.supplyCurrent.valueAsDouble)
        Logger.recordOutput("Armavator/armCurrent", armMotor.supplyCurrent.valueAsDouble)
        Logger.recordOutput("Armavator/pivotCurrent", pivotMotor.supplyCurrent.valueAsDouble)

        Logger.recordOutput("Armavator/elevatorVelocity", elevatorMotor.velocity.valueAsDouble)
        Logger.recordOutput("Armavator/armVelocity", armMotor.velocity.valueAsDouble)
        Logger.recordOutput("Armavator/pivotVelocity", pivotMotor.velocity.valueAsDouble)

        Logger.recordOutput("Armavator/elevatorFeedforward", elevatorFeedforward)
        Logger.recordOutput("Armavator/armFeedforward", armFeedForward)
        Logger.recordOutput("Armavator/pivotFeedforward", pivotFeedForward)

        Logger.recordOutput("Armavator/elevatorEncoderHeight", elevatorEncoderHeight.asInches)
        Logger.recordOutput("Armavator/rawElevatorEncoderValue", rawElevatorEncoderValue)
//        if ((elevatorEncoderHeight.asInches - currentHeight.asInches).absoluteValue > 0.5){
//            elevatorMotor.setPosition(elevatorEncoderHeight.asInches * ELEVATOR_REVOLUTIONS_PER_INCH)
//        }

        if (periodicFeedForward) {
            heightSetpoint = heightSetpoint
            armAngleSetpoint = armAngleSetpoint
            pivotAngleSetpoint = pivotAngleSetpoint
        }


        LoopLogger.record("Armavator pirdc")
    }

    /**
    percent is a double between -1 and 1
     **/
    fun setElevatorPercentOut(percent: Double) {
        elevatorMotor.setControl(DutyCycleOut(percent))
        println("elevator percentage: $percent")
    }

    fun goToPose(pose: Pose, isFlipped: Boolean = false) {
        val targetPose = if (isFlipped) pose.reflect() else pose
        heightSetpoint = targetPose.elevatorHeight
        armAngleSetpoint = targetPose.armAngle
        pivotAngleSetpoint = targetPose.pivotAngle
    }
}