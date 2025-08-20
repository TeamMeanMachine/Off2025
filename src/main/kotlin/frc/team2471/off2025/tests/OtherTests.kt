package frc.team2471.off2025.tests

import edu.wpi.first.wpilibj.Timer.delay
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.team2471.off2025.Armavator
import frc.team2471.off2025.Pose
import frc.team2471.off2025.util.units.degrees


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
