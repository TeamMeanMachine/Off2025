package frc.team2471.off2025.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicDutyCycle
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc2025.CANivores
import org.team2471.frc2025.Falcons

object Elevator: SubsystemBase() {
    //TODO: Create elevator motor


    val revolutionsPerInch = 1.6


    //TODO: Get height of elevator from motor rotations using a variable getter and the revolutions per inch ratio


    val MIN_HEIGHT_INCHES = 0.0
    val MAX_HEIGHT_INCHES = 58.0

    init {
        val motorConfigs = TalonFXConfiguration().apply {
            //TODO: Set safe current limits
            //TODO: Invert leader and/or followers correctly
            //TODO: Set kP and kG values
            //TODO: Configure MotionMagic
        }

        //TODO: apply configs to all motors

        //TODO: Tell motors to follow leader
    }


    fun setPercentOut(percent: Double) {
        //TODO: Set percent output of motors
    }

    fun setPosition(inches: Double) {
        //TODO: Set position of motors
    }


    //TODO: Make a set motionMagic function that sets position using motionMagic



}