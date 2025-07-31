package frc.team2471.off2025.util.localization

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.util.struct.Struct
import java.nio.ByteBuffer

/**
 * PoseEstimateStruct is a class that implements the Struct interface for the PoseEstimate type. It
 * provides methods to get the type class, type name, size, schema, and nested structures. It also
 * provides methods to pack and unpack PoseEstimate objects to and from ByteBuffers.
 *
 *
 * The PoseEstimateStruct class is immutable and ensures that the PoseEstimate objects are
 * correctly serialized and deserialized with the appropriate schema.
 *
 *
 * Methods:
 *
 *
 *  * [.getTypeClass] - Returns the class type of PoseEstimate.
 *  * [.getTypeName] - Returns the name of the type as a string.
 *  * [.getSize] - Returns the size of the PoseEstimate structure in bytes.
 *  * [.getSchema] - Returns the schema of the PoseEstimate structure as a string.
 *  * [.getNested] - Returns an array of nested Structs within the PoseEstimate
 * structure.
 *  * [.unpack] - Unpacks a ByteBuffer into a PoseEstimate object.
 *  * [.pack] - Packs a PoseEstimate object into a ByteBuffer.
 *  * [.isImmutable] - Returns true indicating that the structure is immutable.
 *
 */
class PoseEstimateStruct : Struct<PoseEstimate> {
    override fun getTypeClass(): Class<PoseEstimate> {
        return PoseEstimate::class.java
    }

    override fun getTypeName(): String {
        return "PoseEstimate"
    }

    override fun getSize(): Int {
        return Pose2d.struct.size + Struct.kSizeInt32 * 2
    }

    override fun getSchema(): String {
        return "int32 id;" + "Pose2d pose;" + "int32 hasVision;"
    }

    override fun getNested(): Array<Struct<*>> {
        return arrayOf(Pose2d.struct)
    }

    override fun unpack(bb: ByteBuffer): PoseEstimate {
        val id = bb.getInt()
        val pose = Pose2d.struct.unpack(bb)
        val hasVision = bb.getInt() != 0
        return PoseEstimate(id, pose, hasVision)
    }


    override fun pack(bb: ByteBuffer, value: PoseEstimate) {
        bb.putInt(value.id)
        Pose2d.struct.pack(bb, value.pose)
        bb.putInt(if (value.hasVision()) 1 else 0)
    }

    override fun isImmutable(): Boolean {
        return true
    }
}
