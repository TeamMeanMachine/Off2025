package frc.team2471.off2025.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityDutyCycle
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command
import org.team2471.frc2025.Falcons

// By making a subsystem a Kotlin object, we ensure there is only ever one instance of it.
// It also reduces the need to have reference variables for the subsystems to be passed around.
object Shooter : SubsystemBase() {
    val table = NetworkTableInstance.getDefault().getTable("Shooter")
    val shooterVelocityEntry = table.getEntry("Velocity")
    val shooterVoltsEntry = table.getEntry("Volts")
    val shooterVelocitySetpointEntry = table.getEntry("Velocity Setpoint")

    const val MAX_VELOCITY = 4000.0
    val shooterMotorTop = TalonFX(Falcons.SHOOTER_TOP)

    var shooterVelocitySetpoint: Double = 0.0
        set(value) {
            shooterMotorTop.setControl(VelocityVoltage(value/60.0))
            field = value
        }
    val shooterMotorTopVelocity
    get() = shooterMotorTop.velocity.valueAsDouble * 60.0

    init {
        shooterMotorTop.configurator.apply(TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimit = 30.0
                SupplyCurrentLimitEnable = true
            }
            Slot0.kP = 0.144
            Slot0.kD = 0.0072
            Slot0.kV = 0.116
            Slot0.kS = 0.0
        })
    }

    fun setVelocity(rpm: Double) {
        shooterMotorTop.setControl(VelocityDutyCycle(rpm / 60.0))
    }

    fun setVoltage(volts: Double) {
        println("setting voltage $volts")
        shooterMotorTop.setControl(VoltageOut(volts))
    }

    fun stop() {
        println("stopping")
        setVoltage(0.0)
    }

    override fun periodic() {
        shooterVoltsEntry.setDouble(shooterMotorTop.motorVoltage.valueAsDouble)
        shooterVelocityEntry.setDouble(shooterMotorTopVelocity)
        shooterVelocitySetpointEntry.setDouble(shooterVelocitySetpoint)
    }

    override fun simulationPeriodic()
    {
        // This method will be called once per scheduler run during simulation
    }

    fun exampleAction()
    {
        // This action is called by the ExampleCommand
        println("ExampleSubsystem.exampleAction has been called")
    }
}