package com.radiomaster.egogcs.ui

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * 可热插拔参数化矢量无人机模型定义层 (Plug-and-Play Vector Drone Rig)：
 *
 * 坐标系约定 (标准 ROS REP-103 机体 FLU 归一化坐标系，单位 1.0 = 机臂展开半径)：
 *   +X = 机头正前方 (Forward / Nose)
 *   +Y = 机身左侧   (Port / Left)
 *   +Z = 机身正上方 (Up / Top)
 *
 * 已内置适配用户 `Friendzhb/tilt-uav-indi` 与 `mpc_ros1_for_px4` 的真实物理构型：
 *   - 4 无刷电机 (M1 右前, M2 左后, M3 左前, M4 右后) + 2 沿机体 Yb 轴倾转舵机；
 *   - 舵机 1 (servo_1 / servo_f) 驱动前组电机 M1、M3 同步前倾/后倾；
 *   - 舵机 2 (servo_2 / servo_r) 驱动后组电机 M2、M4 同步前倾/后倾。
 */
object DroneVectorModel {

    enum class PartRole {
        FUSELAGE_SHELL,
        FUSELAGE_EDGE,
        ARM_BEAM,
        LIDAR_DOME,
        GIMBAL_NOSE,
        NAV_FRONT_RED,
        NAV_REAR_GREEN,
        LANDING_SKID,
        TILT_SERVO_SHAFT,
        THRUST_VECTOR
    }

    data class VecSeg3D(
        val x0: Float,
        val y0: Float,
        val z0: Float,
        val x1: Float,
        val y1: Float,
        val z1: Float,
        val role: PartRole,
        val widthFactor: Float = 1.0f
    )

    data class VecFace3D(
        val pts: FloatArray,
        val role: PartRole,
        val shadeBoost: Float = 1.0f
    )

    /**
     * 旋翼/倾转短舱安装位：
     * @param servoGroup 0=固定不可倾转, 1=舵机1控制组(前组 M1/M3), 2=舵机2控制组(后组 M2/M4)
     * @param motorId PX4 标准电机编号 1..4
     */
    data class RotorMount(
        val cx: Float,
        val cy: Float,
        val cz: Float,
        val radius: Float,
        val isFront: Boolean,
        val spinSign: Int,
        val servoGroup: Int = 0,
        val motorId: Int = 1
    )

    class VectorRigSpec(
        val modelName: String,
        val isTiltRotor: Boolean,
        val faces: List<VecFace3D>,
        val segments: List<VecSeg3D>,
        val rotors: List<RotorMount>
    )

    val rigTiltQuad: VectorRigSpec = buildTiltIndiQuadRig()
    val rigStandardQuad: VectorRigSpec = buildPlaceholderTacticalQuad()

    @Volatile
    var activeRig: VectorRigSpec = rigTiltQuad

    fun rigFor(tiltEnabled: Boolean): VectorRigSpec =
        if (tiltEnabled) rigTiltQuad else rigStandardQuad

    /**
     * 将机体 FLU 坐标点 (bx, by, bz) 按实时三维姿态 (roll, pitch, yaw) 刚体变换至世界局部系 (outXyz[0..2])。
     */
    fun transformBodyToWorld(
        bx: Float,
        by: Float,
        bz: Float,
        scale: Float,
        roll: Float,
        pitch: Float,
        yaw: Float,
        outXyz: FloatArray
    ) {
        val sx = bx * scale
        val sy = by * scale
        val sz = bz * scale

        val cr = cos(roll)
        val sr = sin(roll)
        val y1 = sy * cr - sz * sr
        val z1 = sy * sr + sz * cr

        val cp = cos(pitch)
        val sp = sin(pitch)
        val x2 = sx * cp - z1 * sp
        val z2 = sx * sp + z1 * cp

        val cy = cos(yaw)
        val sYaw = sin(yaw)
        outXyz[0] = x2 * cy - y1 * sYaw
        outXyz[1] = x2 * sYaw + y1 * cy
        outXyz[2] = z2
    }

    /**
     * 将位于倾转旋翼短舱局部系下的点 (lx, ly, lz) 先绕铰链中心 (cx, cy, cz) 沿机体 Yb 轴倾转 [tiltRad]
     * (tiltRad > 0 表示推力轴向机头 +Xb 前倾产生正向水平推力 Fx = T*sin(alpha))，再变换至世界局部系。
     */
    fun transformNacellePointToWorld(
        cx: Float,
        cy: Float,
        cz: Float,
        lx: Float,
        ly: Float,
        lz: Float,
        tiltRad: Float,
        scale: Float,
        roll: Float,
        pitch: Float,
        yaw: Float,
        outXyz: FloatArray
    ) {
        val ct = cos(tiltRad)
        val st = sin(tiltRad)
        val bx = cx + lx * ct + lz * st
        val by = cy + ly
        val bz = cz - lx * st + lz * ct
        transformBodyToWorld(bx, by, bz, scale, roll, pitch, yaw, outXyz)
    }

    fun installCustom2DOutlineRig(
        name: String,
        polylines2D: List<FloatArray>,
        rotors: List<RotorMount>,
        halfThicknessZ: Float = 0.07f
    ) {
        val segs = ArrayList<VecSeg3D>()
        for (poly in polylines2D) {
            val n = poly.size / 2
            for (i in 0 until n - 1) {
                val x0 = poly[i * 2]
                val y0 = poly[i * 2 + 1]
                val x1 = poly[(i + 1) * 2]
                val y1 = poly[(i + 1) * 2 + 1]
                segs.add(VecSeg3D(x0, y0, halfThicknessZ, x1, y1, halfThicknessZ, PartRole.FUSELAGE_EDGE, 1.15f))
                segs.add(VecSeg3D(x0, y0, -halfThicknessZ, x1, y1, -halfThicknessZ, PartRole.FUSELAGE_EDGE, 0.85f))
                if (i % 2 == 0) {
                    segs.add(VecSeg3D(x0, y0, -halfThicknessZ, x0, y0, halfThicknessZ, PartRole.ARM_BEAM, 0.75f))
                }
            }
        }
        activeRig = VectorRigSpec(name, false, emptyList(), segs, rotors)
    }

    private fun appendCommonHullAndSensors(faces: MutableList<VecFace3D>, segs: MutableList<VecSeg3D>) {
        val topZ = 0.085f
        val botZ = -0.065f
        val hullXY = arrayOf(
            floatArrayOf(0.42f, 0.11f),
            floatArrayOf(0.18f, 0.22f),
            floatArrayOf(-0.20f, 0.20f),
            floatArrayOf(-0.36f, 0.10f),
            floatArrayOf(-0.36f, -0.10f),
            floatArrayOf(-0.20f, -0.20f),
            floatArrayOf(0.18f, -0.22f),
            floatArrayOf(0.42f, -0.11f)
        )

        val topDeckPts = FloatArray(hullXY.size * 3)
        val botDeckPts = FloatArray(hullXY.size * 3)
        for (i in hullXY.indices) {
            topDeckPts[i * 3] = hullXY[i][0]
            topDeckPts[i * 3 + 1] = hullXY[i][1]
            topDeckPts[i * 3 + 2] = topZ

            val rev = hullXY.lastIndex - i
            botDeckPts[i * 3] = hullXY[rev][0] * 0.90f
            botDeckPts[i * 3 + 1] = hullXY[rev][1] * 0.90f
            botDeckPts[i * 3 + 2] = botZ
        }
        faces.add(VecFace3D(topDeckPts, PartRole.FUSELAGE_SHELL, 1.05f))
        faces.add(VecFace3D(botDeckPts, PartRole.FUSELAGE_SHELL, 0.65f))

        for (i in hullXY.indices) {
            val nxt = (i + 1) % hullXY.size
            val x0 = hullXY[i][0]
            val y0 = hullXY[i][1]
            val x1 = hullXY[nxt][0]
            val y1 = hullXY[nxt][1]
            segs.add(VecSeg3D(x0, y0, topZ, x1, y1, topZ, PartRole.FUSELAGE_EDGE, 1.25f))
            segs.add(VecSeg3D(x0 * 0.90f, y0 * 0.90f, botZ, x1 * 0.90f, y1 * 0.90f, botZ, PartRole.FUSELAGE_EDGE, 0.85f))
            segs.add(VecSeg3D(x0, y0, topZ, x0 * 0.90f, y0 * 0.90f, botZ, PartRole.FUSELAGE_EDGE, 0.9f))
        }

        // 机头双目相机视窗与航向指向针
        segs.add(VecSeg3D(0.42f, -0.09f, 0.02f, 0.48f, -0.05f, 0.02f, PartRole.GIMBAL_NOSE, 1.3f))
        segs.add(VecSeg3D(0.42f, 0.09f, 0.02f, 0.48f, 0.05f, 0.02f, PartRole.GIMBAL_NOSE, 1.3f))
        segs.add(VecSeg3D(0.48f, -0.05f, 0.02f, 0.48f, 0.05f, 0.02f, PartRole.GIMBAL_NOSE, 1.4f))
        segs.add(VecSeg3D(0.45f, 0f, 0.02f, 0.88f, 0f, 0.02f, PartRole.GIMBAL_NOSE, 1.5f))
        segs.add(VecSeg3D(0.88f, 0f, 0.02f, 0.74f, 0.08f, 0.02f, PartRole.GIMBAL_NOSE, 1.35f))
        segs.add(VecSeg3D(0.88f, 0f, 0.02f, 0.74f, -0.08f, 0.02f, PartRole.GIMBAL_NOSE, 1.35f))

        // 顶部 FAST-LIO 固态激光雷达塔
        val lidarBaseZ = topZ
        val lidarTopZ = 0.21f
        val lidarR = 0.13f
        for (k in 0 until 8) {
            val a0 = (k / 8.0) * PI * 2.0
            val a1 = ((k + 1) / 8.0) * PI * 2.0
            val lx0 = -0.03f + cos(a0).toFloat() * lidarR
            val ly0 = sin(a0).toFloat() * lidarR
            val lx1 = -0.03f + cos(a1).toFloat() * lidarR
            val ly1 = sin(a1).toFloat() * lidarR
            segs.add(VecSeg3D(lx0, ly0, lidarTopZ, lx1, ly1, lidarTopZ, PartRole.LIDAR_DOME, 1.15f))
            if (k % 2 == 0) {
                segs.add(VecSeg3D(lx0, ly0, lidarBaseZ, lx0, ly0, lidarTopZ, PartRole.LIDAR_DOME, 0.95f))
            }
        }

        // 双橇式起落架
        for (side in intArrayOf(-1, 1)) {
            val sy = side * 0.24f
            val skidZ = -0.22f
            segs.add(VecSeg3D(-0.26f, sy, skidZ, 0.28f, sy, skidZ, PartRole.LANDING_SKID, 1.15f))
            segs.add(VecSeg3D(0.28f, sy, skidZ, 0.34f, sy, skidZ + 0.05f, PartRole.LANDING_SKID, 1.1f))
            segs.add(VecSeg3D(0.14f, side * 0.16f, botZ, 0.16f, sy, skidZ, PartRole.LANDING_SKID, 0.95f))
            segs.add(VecSeg3D(-0.14f, side * 0.16f, botZ, -0.16f, sy, skidZ, PartRole.LANDING_SKID, 0.95f))
        }
    }

    /**
     * 用户 `Friendzhb/tilt-uav-indi` (g1_indi_xyz) 专属四电机 + 双 Yb 轴倾转舵机矢量四旋翼构型：
     * - 舵机 1 横向联动轴 (x = +0.62): 驱动 M1 (右前 CCW) 与 M3 (左前 CW) 同步前/后倾转；
     * - 舵机 2 横向联动轴 (x = -0.62): 驱动 M2 (左后 CCW) 与 M4 (右后 CW) 同步前/后倾转；
     * - 旋翼护圈、桨叶与三维推力矢量箭将在渲染层随实时倾转角 (alpha_1, alpha_2) 动态物理偏转！
     */
    private fun buildTiltIndiQuadRig(): VectorRigSpec {
        val faces = ArrayList<VecFace3D>()
        val segs = ArrayList<VecSeg3D>()
        appendCommonHullAndSensors(faces, segs)

        val motorZ = 0.062f
        val frontX = 0.62f
        val rearX = -0.62f
        val spanY = 0.64f

        // 四根主承力碳纤斜臂 + 轴承支座
        val armSpecs = arrayOf(
            // (rootX, rootY, tipX, tipY, isFront, spinSign, servoGroup, motorId)
            floatArrayOf(0.18f, -0.16f, frontX, -spanY, 1f, 1f, 1f, 1f), // M1 右前 (PX4 #1) -> 舵机 1
            floatArrayOf(-0.18f, 0.16f, rearX, spanY, 0f, 1f, 2f, 2f), // M2 左后 (PX4 #2) -> 舵机 2
            floatArrayOf(0.18f, 0.16f, frontX, spanY, 1f, -1f, 1f, 3f), // M3 左前 (PX4 #3) -> 舵机 1
            floatArrayOf(-0.18f, -0.16f, rearX, -spanY, 0f, -1f, 2f, 4f) // M4 右后 (PX4 #4) -> 舵机 2
        )

        val rotors = ArrayList<RotorMount>(4)
        val shroudRadius = 0.35f

        for (spec in armSpecs) {
            val rx = spec[0]
            val ry = spec[1]
            val tx = spec[2]
            val ty = spec[3]
            val isFront = spec[4] > 0.5f
            val spin = spec[5].toInt()
            val sGrp = spec[6].toInt()
            val mId = spec[7].toInt()

            segs.add(VecSeg3D(rx, ry, 0.048f, tx, ty, motorZ, PartRole.ARM_BEAM, 1.48f))
            segs.add(VecSeg3D(rx * 0.92f, ry * 0.92f, -0.02f, tx, ty, motorZ - 0.038f, PartRole.ARM_BEAM, 1.0f))
            // 倾转轴承耳座
            segs.add(VecSeg3D(tx, ty - 0.045f, motorZ, tx, ty + 0.045f, motorZ, PartRole.TILT_SERVO_SHAFT, 1.55f))

            rotors.add(
                RotorMount(
                    cx = tx,
                    cy = ty,
                    cz = motorZ,
                    radius = shroudRadius * 0.88f,
                    isFront = isFront,
                    spinSign = spin,
                    servoGroup = sGrp,
                    motorId = mId
                )
            )
        }

        // 前组舵机 1 (控制 M1/M3) 与后组舵机 2 (控制 M2/M4) 横向碳纤同步倾转连杆轴 + 中置数字舵机舱
        segs.add(VecSeg3D(frontX, -spanY, motorZ, frontX, spanY, motorZ, PartRole.TILT_SERVO_SHAFT, 1.35f))
        segs.add(VecSeg3D(rearX, -spanY, motorZ, rearX, spanY, motorZ, PartRole.TILT_SERVO_SHAFT, 1.35f))
        // 前后舵机安装支臂 (连接机身前后端与前后倾转连杆中心)
        segs.add(VecSeg3D(0.38f, 0f, 0.05f, frontX, 0f, motorZ, PartRole.TILT_SERVO_SHAFT, 1.25f))
        segs.add(VecSeg3D(-0.35f, 0f, 0.05f, rearX, 0f, motorZ, PartRole.TILT_SERVO_SHAFT, 1.25f))
        // 舵机壳体微框 (S1 前舵机 & S2 后舵机)
        segs.add(VecSeg3D(frontX - 0.04f, -0.07f, motorZ + 0.02f, frontX + 0.04f, 0.07f, motorZ + 0.02f, PartRole.TILT_SERVO_SHAFT, 1.3f))
        segs.add(VecSeg3D(rearX - 0.04f, -0.07f, motorZ + 0.02f, rearX + 0.04f, 0.07f, motorZ + 0.02f, PartRole.TILT_SERVO_SHAFT, 1.3f))

        return VectorRigSpec(
            modelName = "TILT-INDI 4M2S 双舵矢量机",
            isTiltRotor = true,
            faces = faces,
            segments = segs,
            rotors = rotors
        )
    }

    /**
     * 常规固定桨盘四旋翼基准模型 (用于对比或切换常规机型)
     */
    private fun buildPlaceholderTacticalQuad(): VectorRigSpec {
        val faces = ArrayList<VecFace3D>()
        val segs = ArrayList<VecSeg3D>()
        appendCommonHullAndSensors(faces, segs)

        val armSpecs = arrayOf(
            floatArrayOf(0.17f, -0.16f, 0.64f, -0.64f, 1f, 1f, 1f),
            floatArrayOf(-0.17f, 0.15f, -0.64f, 0.64f, 0f, 1f, 2f),
            floatArrayOf(0.17f, 0.16f, 0.64f, 0.64f, 1f, -1f, 3f),
            floatArrayOf(-0.17f, -0.15f, -0.64f, -0.64f, 0f, -1f, 4f)
        )

        val rotors = ArrayList<RotorMount>(4)
        val shroudRadius = 0.36f

        for (spec in armSpecs) {
            val rx = spec[0]
            val ry = spec[1]
            val tx = spec[2]
            val ty = spec[3]
            val isFront = spec[4] > 0.5f
            val spin = spec[5].toInt()
            val mId = spec[6].toInt()
            val motorZ = 0.055f

            segs.add(VecSeg3D(rx, ry, 0.045f, tx, ty, motorZ, PartRole.ARM_BEAM, 1.45f))
            segs.add(VecSeg3D(rx * 0.92f, ry * 0.92f, -0.02f, tx, ty, motorZ - 0.04f, PartRole.ARM_BEAM, 1.0f))
            segs.add(VecSeg3D(tx, ty, motorZ - 0.045f, tx, ty, motorZ + 0.045f, PartRole.FUSELAGE_EDGE, 1.3f))

            val ringRole = if (isFront) PartRole.NAV_FRONT_RED else PartRole.NAV_REAR_GREEN
            for (s in 0 until 12) {
                val a0 = (s / 12.0) * PI * 2.0
                val a1 = ((s + 1) / 12.0) * PI * 2.0
                segs.add(
                    VecSeg3D(
                        tx + cos(a0).toFloat() * shroudRadius,
                        ty + sin(a0).toFloat() * shroudRadius,
                        motorZ + 0.02f,
                        tx + cos(a1).toFloat() * shroudRadius,
                        ty + sin(a1).toFloat() * shroudRadius,
                        motorZ + 0.02f,
                        ringRole,
                        1.2f
                    )
                )
            }

            rotors.add(RotorMount(tx, ty, motorZ + 0.032f, shroudRadius * 0.86f, isFront, spin, 0, mId))
        }

        return VectorRigSpec(
            modelName = "TACTICAL-X4 常规四旋翼",
            isTiltRotor = false,
            faces = faces,
            segments = segs,
            rotors = rotors
        )
    }
}
