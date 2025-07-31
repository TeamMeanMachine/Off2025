package frc.team2471.off2025.util.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N8
import frc.team2471.off2025.util.vision.Fiducial

/** Describes a given vision camera pipeline configuration.  */
class PipelineConfig(
    @JvmField val fiducialType: Fiducial.Type,
// pixels
    @JvmField val imageWidth: Int,
    // pixels
    @JvmField val imageHeight: Int,
    // used for sim only
    @JvmField val camIntrinsics: Matrix<N3, N3>,
    // used for sim only
    @JvmField val distCoeffs: Matrix<N8, N1>
) {
//    init {
//        this.distCoeffs = distCoeffs
//    }
}
