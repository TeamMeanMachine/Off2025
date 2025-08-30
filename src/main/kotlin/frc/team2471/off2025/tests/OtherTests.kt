package frc.team2471.off2025.tests

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveSteerGains
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Timer.delay
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team2471.off2025.Armavator
import frc.team2471.off2025.Pose
import frc.team2471.off2025.util.beforeWait
import frc.team2471.off2025.util.sequenceCommand
import frc.team2471.off2025.util.units.degrees
import frc.team2471.off2025.util.units.seconds
import frc.team2471.off2025.util.units.volts
import frc.team2471.off2025.util.units.voltsPerSecond
import org.littletonrobotics.junction.Logger


fun Armavator.pivotSetpointTest(): Command {
    return Commands.run({
        goToPose(Pose.DRIVE)
        goToPose(Pose(Pose.current.elevatorHeight, Pose.current.armAngle, 180.0.degrees))
        goToPose(Pose.DRIVE)
        goToPose(Pose(Pose.current.elevatorHeight, Pose.current.armAngle, (-180.0).degrees))
        goToPose(Pose.DRIVE)
    }, Armavator)
}

fun Armavator.armSetpointTest(): Command {
    return Commands.run({
        goToPose(Pose.DRIVE)
        goToPose(Pose(Pose.current.elevatorHeight, Pose.INTAKE_GROUND.armAngle, Pose.current.pivotAngle))
        goToPose(Pose.DRIVE)
        goToPose(Pose(Pose.current.elevatorHeight, -Pose.INTAKE_GROUND.armAngle, Pose.current.pivotAngle))
        goToPose(Pose.DRIVE)
    }, Armavator)

}

fun Armavator.elevatorSetpointTest(): Command {
    return Commands.run({
        println("elevator setpoint testing!!!!!!!!!")
        goToPose(Pose.DRIVE)
        delay(0.25)
        goToPose(Pose(Pose.SCORE_L3.elevatorHeight, Pose.current.armAngle, Pose.current.pivotAngle))
//        delay(0.25)
//        goToPose(Pose(Pose.SCORE_L4.elevatorHeight, Pose.current.armAngle, Pose.current.pivotAngle))
//        delay(0.25)
//        goToPose(Pose(Pose.SCORE_L3.elevatorHeight, Pose.current.armAngle, Pose.current.pivotAngle))
        delay(5.0)
//        goToPose(Pose.DRIVE)
        }, Armavator)
}

fun Armavator.sysIDPivot(): Command {
    /** Used to find steer motor PID and SVA constants. */
    val sysIDRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            0.5.voltsPerSecond,
            7.0.volts,
            10.0.seconds
        ) { state: SysIdRoutineLog.State ->
            SignalLogger.writeString("SysIdPivot_State", state.toString())
            Logger.recordOutput("SysId_Pivot", state.toString())
        },
        Mechanism({ volts: Voltage? -> pivotMotor.setControl(VoltageOut(volts))}, null, this)
    )
    return sequenceCommand(
        sysIDRoutine.dynamic(SysIdRoutine.Direction.kForward).beforeWait(1.0),
                sysIDRoutine.dynamic(SysIdRoutine.Direction.kReverse).beforeWait(1.0),
        sysIDRoutine.quasistatic(SysIdRoutine.Direction.kForward).beforeWait(1.0),
        sysIDRoutine.quasistatic(SysIdRoutine.Direction.kReverse).beforeWait(1.0)
    )
}
