package frc.team2471.off2025.util

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.button.CommandXboxController

inline val ChassisSpeeds.translation: Translation2d get() = Translation2d(this.vxMetersPerSecond, this.vyMetersPerSecond)
fun ChassisSpeeds.fieldToRobotCentric(heading: Rotation2d): ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(this, heading)
fun ChassisSpeeds.robotToFieldCentric(heading: Rotation2d): ChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(this, heading)
fun SwerveDriveKinematics.toChassisSpeedsK(speeds: Array<SwerveModuleState>): ChassisSpeeds {
    return this.toChassisSpeeds(*speeds) //Intellij thinks this is an error. which is lame...
}

/** Sometimes the sim GUI doesn't detect an Xbox controller as a gamepad and does not bind it as such. [simBeingDumb] rebinds the joystick as if the "map gamepad" button was pressed. */
class MeanCommandXboxController(port: Int, val simBeingDumb: Boolean = false): CommandXboxController(port) {
    override fun getRightX(): Double {
        if (simBeingDumb) {
            return super.leftTriggerAxis
        }
        return super.getRightX()
    }

    override fun getLeftTriggerAxis(): Double {
        if (simBeingDumb) {
            return super.rightX
        }
        return super.leftTriggerAxis
    }
}
