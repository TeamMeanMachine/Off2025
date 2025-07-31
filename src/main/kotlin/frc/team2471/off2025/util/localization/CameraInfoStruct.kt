package frc.team2471.off2025.util.localization

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N8
import edu.wpi.first.util.struct.Struct
import frc.team2471.off2025.util.localization.CameraInfo
import java.nio.ByteBuffer
import java.util.*

/**
 * CameraInfoStruct is a structure that implements the Struct interface for the CameraInfo class. It
 * provides methods to get the type class, type name, size, schema, nested structures, and to pack
 * and unpack CameraInfo objects from a ByteBuffer.
 *
 *
 * The structure includes the following fields:
 *
 *
 *  * Transform3d transform
 *  * int32 hasCameraMatrix
 *  * double M00, M01, M02, M10, M11, M12, M20, M21, M22
 *  * int32 hasDistCoeffs
 *  * double d0, d1, d2, d3, d4, d5, d6, d7
 *
 *
 *
 * The unpack method reads data from a ByteBuffer and constructs a CameraInfo object. The pack
 * method writes a CameraInfo object to a ByteBuffer.
 *
 *
 * The isImmutable method indicates that the structure is immutable.
 *
 *
 * Used by NetworkTables to efficiently transmit camera calibration data between robot and driver
 * station.
 *
 * @see ByteBuffer
 *
 * @see frc.team2471.off2025.util.localization.CameraInfo
 *
 * @see Transform3d
 *
 * @see Struct
 */
class CameraInfoStruct : Struct<CameraInfo> {
    override fun getTypeClass(): Class<CameraInfo> {
        return CameraInfo::class.java
    }

    override fun getTypeName(): String {
        return "CameraInfo"
    }

    override fun getSize(): Int {
        return Transform3d.struct.size + Struct.kSizeInt32 * 2 + Struct.kSizeDouble * 17
    }

    override fun getSchema(): String {
        return ("Transform3d transform;"
                + "int32 hasCameraMatrix;"
                + "double M00;"
                + "double M01;"
                + "double M02;"
                + "double M10;"
                + "double M11;"
                + "double M12;"
                + "double M20;"
                + "double M21;"
                + "double M22;"
                + "int32 hasDistCoeffs;"
                + "double d0;"
                + "double d1;"
                + "double d2;"
                + "double d3;"
                + "double d4;"
                + "double d5;"
                + "double d6;"
                + "double d7;")
    }

    override fun getNested(): Array<Struct<*>> {
        return arrayOf(Transform3d.struct)
    }

    /**
     * Unpacks camera information from a ByteBuffer into a CameraInfo object.
     *
     * @param bb The ByteBuffer containing the packed camera information data
     * @return A new CameraInfo object containing the unpacked camera transformation, optional camera
     * matrix, and optional distortion coefficients
     *
     * The ByteBuffer should contain, in order: - A Transform3d struct - A boolean flag for
     * camera matrix presence - Nine doubles representing a 3x3 camera matrix (if present) - A
     * boolean flag for distortion coefficients presence - Eight doubles representing distortion
     * coefficients (if present)
     */
    override fun unpack(bb: ByteBuffer): CameraInfo {
        val transform = Transform3d.struct.unpack(bb)
        val hasCameraMatrix = bb.getInt() != 0
        val M00 = bb.getDouble()
        val M01 = bb.getDouble()
        val M02 = bb.getDouble()
        val M10 = bb.getDouble()
        val M11 = bb.getDouble()
        val M12 = bb.getDouble()
        val M20 = bb.getDouble()
        val M21 = bb.getDouble()
        val M22 = bb.getDouble()
        val hasDistCoeffs = bb.getInt() != 0
        val d0 = bb.getDouble()
        val d1 = bb.getDouble()
        val d2 = bb.getDouble()
        val d3 = bb.getDouble()
        val d4 = bb.getDouble()
        val d5 = bb.getDouble()
        val d6 = bb.getDouble()
        val d7 = bb.getDouble()
        return CameraInfo(
            transform,
            if (hasCameraMatrix)
                Optional.of(
                    MatBuilder.fill(Nat.N3(), Nat.N3(), M00, M01, M02, M10, M11, M12, M20, M21, M22)
                )
            else
                Optional.empty<Matrix<N3, N3>>(),
            if (hasDistCoeffs)
                Optional.of(
                    MatBuilder.fill(
                        Nat.N8(),
                        Nat.N1(),
                        d0,
                        d1,
                        d2,
                        d3,
                        d4,
                        d5,
                        d6,
                        d7
                    )
                )
            else
                Optional.empty<Matrix<N8, N1>>()
        )
    }

    /**
     * Packs camera information into a ByteBuffer. This method serializes a CameraInfo object,
     * including its transformation matrix, camera matrix (if present), and distortion coefficients
     * (if present).
     *
     * @param bb The ByteBuffer to pack the data into
     * @param value The CameraInfo object containing the data to be packed
     */
    override fun pack(bb: ByteBuffer, value: CameraInfo) {
        Transform3d.struct.pack(bb, value.transform)
        val hasCameraMatrix = value.cameraMatrix!!.isPresent()
        bb.putInt(if (hasCameraMatrix) 1 else 0)
        bb.putDouble(if (hasCameraMatrix) value.cameraMatrix.get().get(0, 0) else 0.0)
        bb.putDouble(if (hasCameraMatrix) value.cameraMatrix.get().get(0, 1) else 0.0)
        bb.putDouble(if (hasCameraMatrix) value.cameraMatrix.get().get(0, 2) else 0.0)
        bb.putDouble(if (hasCameraMatrix) value.cameraMatrix.get().get(1, 0) else 0.0)
        bb.putDouble(if (hasCameraMatrix) value.cameraMatrix.get().get(1, 1) else 0.0)
        bb.putDouble(if (hasCameraMatrix) value.cameraMatrix.get().get(1, 2) else 0.0)
        bb.putDouble(if (hasCameraMatrix) value.cameraMatrix.get().get(2, 0) else 0.0)
        bb.putDouble(if (hasCameraMatrix) value.cameraMatrix.get().get(2, 1) else 0.0)
        bb.putDouble(if (hasCameraMatrix) value.cameraMatrix.get().get(2, 2) else 0.0)
        val hasDistCoeffs = value.distCoeffs!!.isPresent()
        bb.putInt(if (hasDistCoeffs) 1 else 0)
        bb.putDouble(if (hasDistCoeffs) value.distCoeffs.get().get(0, 0) else 0.0)
        bb.putDouble(if (hasDistCoeffs) value.distCoeffs.get().get(1, 0) else 0.0)
        bb.putDouble(if (hasDistCoeffs) value.distCoeffs.get().get(2, 0) else 0.0)
        bb.putDouble(if (hasDistCoeffs) value.distCoeffs.get().get(3, 0) else 0.0)
        bb.putDouble(if (hasDistCoeffs) value.distCoeffs.get().get(4, 0) else 0.0)
        bb.putDouble(if (hasDistCoeffs) value.distCoeffs.get().get(5, 0) else 0.0)
        bb.putDouble(if (hasDistCoeffs) value.distCoeffs.get().get(6, 0) else 0.0)
        bb.putDouble(if (hasDistCoeffs) value.distCoeffs.get().get(7, 0) else 0.0)
    }

    override fun isImmutable(): Boolean {
        return true
    }
}
