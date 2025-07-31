package frc.team2471.off2025.util.vision

import org.photonvision.targeting.PhotonTrackedTarget

/** A data class for a pipeline packet.  */
class PipelineVisionPacket(
    private val m_hasTargets: Boolean,
    /**
     * Gets best target.
     *
     * @return the best target.
     */
    val bestTarget: PhotonTrackedTarget?,
    /**
     * Gets targets.
     *
     * @return the targets.
     */
    val targets: MutableList<PhotonTrackedTarget>?,
    /**
     * Gets the capture timestamp in seconds. -1 if the result has no timestamp set.
     *
     * @return the timestamp in seconds.
     */
    val captureTimestamp: Double
) {
    /**
     * If the vision packet has valid targets.
     *
     * @return if targets are found.
     */
    fun hasTargets(): Boolean {
        return m_hasTargets
    }
}
