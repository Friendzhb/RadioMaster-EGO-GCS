package com.radiomaster.egogcs.ui

import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.sqrt
import org.json.JSONArray
import org.json.JSONObject

/** Reader for the bundled, self-contained GLB: unindexed FLOAT triangles and named tilt nodes. */
internal object VectorGlbAsset {
    data class Part(
        val name: String,
        val group: Int,
        val transform: FloatArray,
        val positionOffset: Int,
        val normalOffset: Int,
        val vertexCount: Int,
        val color: FloatArray
    )

    data class Model(
        val parts: List<Part>,
        val frontPivot: FloatArray,
        val rearPivot: FloatArray,
        val binaryOffset: Int,
        val binaryLength: Int
    )

    fun parse(source: ByteBuffer): Model {
        val buffer = source.duplicate().order(ByteOrder.LITTLE_ENDIAN)
        val size = buffer.limit()
        require(size >= 28 && buffer.getInt(0) == 0x46546c67 && buffer.getInt(4) == 2) { "Not a GLB 2.0 file" }
        require(buffer.getInt(8) == size) { "GLB length mismatch" }
        val jsonLength = buffer.getInt(12)
        require(jsonLength in 2..1_000_000 && 20L + jsonLength + 8 <= size) { "Invalid GLB JSON chunk" }
        require(buffer.getInt(16) == 0x4e4f534a) { "Missing GLB JSON chunk" }
        val text = ByteArray(jsonLength)
        buffer.position(20)
        buffer.get(text)
        val doc = JSONObject(String(text, Charsets.UTF_8).trimEnd(' ', '\u0000'))
        val chunkHeader = 20 + jsonLength
        val binaryLength = buffer.getInt(chunkHeader)
        val binaryOffset = chunkHeader + 8
        require(binaryLength >= 0 && binaryOffset.toLong() + binaryLength == size.toLong()) { "Invalid GLB binary chunk" }
        require(buffer.getInt(chunkHeader + 4) == 0x004e4942) { "Missing GLB binary chunk" }
        val buffers = doc.getJSONArray("buffers")
        require(buffers.length() == 1 && !buffers.getJSONObject(0).has("uri")) { "GLB must be self-contained" }
        require(buffers.getJSONObject(0).getInt("byteLength") <= binaryLength) { "GLB buffer exceeds chunk" }

        val nodes = doc.getJSONArray("nodes")
        val meshes = doc.getJSONArray("meshes")
        val accessors = doc.getJSONArray("accessors")
        val views = doc.getJSONArray("bufferViews")
        val materials = doc.optJSONArray("materials")
        require(nodes.length() in 1..256) { "Invalid node count" }
        val base = (0 until nodes.length()).single { nodes.getJSONObject(it).optString("name") == "base_link" }
        val parts = ArrayList<Part>()
        val seen = BooleanArray(nodes.length())
        var front: FloatArray? = null
        var rear: FloatArray? = null

        fun accessor(index: Int): Pair<Int, Int> {
            val item = accessors.getJSONObject(index)
            require(item.getInt("componentType") == 5126 && item.getString("type") == "VEC3" && !item.has("sparse")) {
                "Only float VEC3 geometry is supported"
            }
            val count = item.getInt("count")
            require(count > 0 && count % 3 == 0 && count <= 2_000_000) { "Invalid vertex count" }
            val view = views.getJSONObject(item.getInt("bufferView"))
            require(view.getInt("buffer") == 0 && view.optInt("byteStride", 12) == 12) { "Unsupported GLB buffer" }
            val within = item.optInt("byteOffset", 0)
            val offset = view.optInt("byteOffset", 0).toLong() + within
            val bytes = count.toLong() * 12
            require(within >= 0 && offset >= 0 && within.toLong() + bytes <= view.getInt("byteLength") && offset + bytes <= binaryLength) {
                "GLB accessor outside buffer"
            }
            require(offset % 4L == 0L) { "Unaligned GLB accessor" }
            return offset.toInt() to count
        }

        fun visit(index: Int, parent: FloatArray, inheritedGroup: Int) {
            require(index in 0 until nodes.length() && !seen[index]) { "Invalid or repeated GLB node" }
            seen[index] = true
            val node = nodes.getJSONObject(index)
            val name = node.optString("name")
            val transform = multiply(parent, nodeMatrix(node))
            val group = when (name) {
                "joint_f" -> {
                    require(front == null && inheritedGroup == 0) { "Invalid front joint" }
                    requireYAxis(node)
                    front = floatArrayOf(transform[12], transform[13], transform[14])
                    1
                }
                "joint_b" -> {
                    require(rear == null && inheritedGroup == 0) { "Invalid rear joint" }
                    requireYAxis(node)
                    rear = floatArrayOf(transform[12], transform[13], transform[14])
                    2
                }
                else -> inheritedGroup
            }
            if (node.has("mesh")) {
                val mesh = meshes.getJSONObject(node.getInt("mesh"))
                val primitives = mesh.getJSONArray("primitives")
                for (i in 0 until primitives.length()) {
                    val primitive = primitives.getJSONObject(i)
                    require(primitive.optInt("mode", 4) == 4 && !primitive.has("indices")) { "Unsupported GLB primitive" }
                    val attrs = primitive.getJSONObject("attributes")
                    val pos = accessor(attrs.getInt("POSITION"))
                    val normal = accessor(attrs.getInt("NORMAL"))
                    require(pos.second == normal.second) { "Position/normal counts differ" }
                    val color = floatArrayOf(0.65f, 0.68f, 0.72f, 1f)
                    val mi = primitive.optInt("material", -1)
                    if (materials != null && mi in 0 until materials.length()) {
                        val factor = materials.getJSONObject(mi).optJSONObject("pbrMetallicRoughness")
                            ?.optJSONArray("baseColorFactor")
                        if (factor != null && factor.length() == 4) {
                            for (c in 0..3) color[c] = factor.getDouble(c).toFloat()
                        }
                    }
                    require(color.all { it.isFinite() && it in 0f..1f }) { "Invalid material color" }
                    parts.add(Part(name, group, transform, pos.first, normal.first, pos.second, color))
                }
            }
            val children = node.optJSONArray("children") ?: JSONArray()
            for (i in 0 until children.length()) visit(children.getInt(i), transform, group)
        }

        visit(base, identity(), 0)
        val frontPivot = requireNotNull(front) { "Missing front joint" }
        val rearPivot = requireNotNull(rear) { "Missing rear joint" }
        require(parts.any { it.group == 0 } && parts.any { it.group == 1 } && parts.any { it.group == 2 }) {
            "Missing UAV geometry"
        }
        return Model(parts, frontPivot, rearPivot, binaryOffset, binaryLength)
    }

    private fun requireYAxis(node: JSONObject) {
        val axis = node.getJSONObject("extras").getJSONArray("axis_local")
        require(axis.length() == 3 && axis.getDouble(0) == 0.0 && axis.getDouble(1) == 1.0 && axis.getDouble(2) == 0.0) {
            "Unsupported tilt axis"
        }
    }

    private fun identity() = floatArrayOf(
        1f, 0f, 0f, 0f,
        0f, 1f, 0f, 0f,
        0f, 0f, 1f, 0f,
        0f, 0f, 0f, 1f
    )

    private fun multiply(a: FloatArray, b: FloatArray): FloatArray {
        val out = FloatArray(16)
        for (c in 0..3) for (r in 0..3) for (k in 0..3) out[c * 4 + r] += a[k * 4 + r] * b[c * 4 + k]
        return out
    }

    private fun nodeMatrix(node: JSONObject): FloatArray {
        require(!node.has("matrix")) { "Matrix nodes are not supported" }
        val translation = vector(node.optJSONArray("translation"), floatArrayOf(0f, 0f, 0f))
        val scale = vector(node.optJSONArray("scale"), floatArrayOf(1f, 1f, 1f))
        val q = vector(node.optJSONArray("rotation"), floatArrayOf(0f, 0f, 0f, 1f))
        val length = sqrt(q.sumOf { (it * it).toDouble() }).toFloat()
        require(length.isFinite() && length > 1e-6f) { "Invalid node quaternion" }
        val x = q[0] / length
        val y = q[1] / length
        val z = q[2] / length
        val w = q[3] / length
        return floatArrayOf(
            (1f - 2f * (y * y + z * z)) * scale[0], (2f * (x * y + z * w)) * scale[0], (2f * (x * z - y * w)) * scale[0], 0f,
            (2f * (x * y - z * w)) * scale[1], (1f - 2f * (x * x + z * z)) * scale[1], (2f * (y * z + x * w)) * scale[1], 0f,
            (2f * (x * z + y * w)) * scale[2], (2f * (y * z - x * w)) * scale[2], (1f - 2f * (x * x + y * y)) * scale[2], 0f,
            translation[0], translation[1], translation[2], 1f
        )
    }

    private fun vector(array: JSONArray?, default: FloatArray): FloatArray {
        if (array == null) return default
        require(array.length() == default.size) { "Invalid GLB transform" }
        return FloatArray(default.size) { array.getDouble(it).toFloat().also { value -> require(value.isFinite()) } }
    }
}
