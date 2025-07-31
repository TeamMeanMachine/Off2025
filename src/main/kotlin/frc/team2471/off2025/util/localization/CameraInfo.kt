package frc.team2471.off2025.util.localization

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N8
import java.util.*

/**
 * The CameraInfo class holds information about a camera's transform, camera matrix, and distortion
 * coefficients.
 *
 *
 * This class is used to encapsulate the camera's intrinsic and extrinsic parameters, which are
 * essential for camera calibration and image processing tasks in robotics.
 *
 *
 * It contains the following fields:
 *
 *
 *  * `m_transform` - The 3D transform representing the camera's position and orientation.
 *  * `m_cameraMatrix` - An optional 3x3 matrix representing the camera's intrinsic
 * parameters.
 *  * `m_distCoeffs` - An optional 8x1 matrix representing the camera's distortion
 * coefficients.
 *
 *
 *
 * The class provides getter methods to access these fields:
 *
 *
 *  * [.getTransform] - Returns the camera's transform.
 *  * [.getCameraMatrix] - Returns the camera's intrinsic matrix, if available.
 *  * [.getDistCoeffs] - Returns the camera's distortion coefficients, if available.
 *
 *
 *
 * Additionally, it includes a static nested class `CameraInfoStruct` for serialization
 * purposes.
 *
 * @param transform The 3D transform representing the camera's position and orientation.
 * @param cameraMatrix An optional 3x3 matrix representing the camera's intrinsic parameters.
 * @param distCoeffs An optional 8x1 matrix representing the camera's distortion coefficients.
 */
class CameraInfo(
    val transform: Transform3d,
    val cameraMatrix: Optional<Matrix<N3, N3>>,
    val distCoeffs: Optional<Matrix<N8, N1>>
) {
    companion object {
        // Struct for serialization.
        @JvmField
        val struct: CameraInfoStruct = CameraInfoStruct()
    }
}
