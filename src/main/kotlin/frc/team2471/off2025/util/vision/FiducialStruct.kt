package frc.team2471.off2025.util.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.util.struct.Struct
import java.nio.ByteBuffer

class FiducialStruct : Struct<Fiducial> {
    override fun getTypeClass(): Class<Fiducial> {
        return Fiducial::class.java
    }

    override fun getTypeName(): String {
        return "Fiducial"
    }

    override fun getSize(): Int {
        return Pose3d.struct.getSize() + Struct.kSizeInt32 * 2 + Struct.kSizeDouble
    }

    override fun getSchema(): String {
        return "int32 type;int32 id;Pose3d pose;double size;"
    }

    override fun getNested(): Array<Struct<*>> {
        return arrayOf<Struct<*>>(Pose3d.struct)
    }

    override fun unpack(bb: ByteBuffer): Fiducial {
        val type = Fiducial.Type.values()[bb.getInt()]
        val id = bb.getInt()
        val pose = Pose3d.struct.unpack(bb)
        val size = bb.getDouble()
        return Fiducial(type, id, pose, size)
    }

    override fun pack(bb: ByteBuffer, value: Fiducial) {
        bb.putInt(value.type.value)
        bb.putInt(value.id())
        Pose3d.struct.pack(bb, value.pose)
        bb.putDouble(value.size)
    }

    override fun isImmutable(): Boolean {
        return true
    }
}
