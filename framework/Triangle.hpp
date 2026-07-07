#pragma once

#include "BVH.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "OBJ_Loader.hpp"
#include "Object.hpp"
#include "Triangle.hpp"
#include <cassert>
#include <array>

bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1,
                          const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{
    Vector3f edge1 = v1 - v0;
    Vector3f edge2 = v2 - v0;
    Vector3f pvec = crossProduct(dir, edge2);
    float det = dotProduct(edge1, pvec);
    if (det == 0 || det < 0)
        return false;

    Vector3f tvec = orig - v0;
    u = dotProduct(tvec, pvec);
    if (u < 0 || u > det)
        return false;

    Vector3f qvec = crossProduct(tvec, edge1);
    v = dotProduct(dir, qvec);
    if (v < 0 || u + v > det)
        return false;

    float invDet = 1 / det;

    tnear = dotProduct(edge2, qvec) * invDet;
    u *= invDet;
    v *= invDet;

    return true;
}

class Triangle : public Object
{
public:
    Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
    Vector3f e1, e2;     // 2 edges v1-v0, v2-v0;
    Vector3f t0, t1, t2; // texture coords
    Vector3f normal;
    Material* m;

    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material* _m = nullptr)
        : v0(_v0), v1(_v1), v2(_v2), m(_m)
    {
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = normalize(crossProduct(e1, e2));
    }

    bool intersect(const Ray& ray) override;
    bool intersect(const Ray& ray, float& tnear,
                   uint32_t& index) const override;
    Intersection getIntersection(Ray ray) override;
    void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
                              const uint32_t& index, const Vector2f& uv,
                              Vector3f& N, Vector2f& st) const override
    {
        N = normal;
        //        throw std::runtime_error("triangle::getSurfaceProperties not
        //        implemented.");
    }
    Vector3f evalDiffuseColor(const Vector2f&) const override;
    Bounds3 getBounds() override;
};

class MeshTriangle : public Object
{
public:
    MeshTriangle(const std::string& filename)
    {
        objl::Loader loader;
        //loader.LoadFile(filename);
		loader.LoadFile(filename);
        assert(loader.LoadedMeshes.size() == 1);
        auto mesh = loader.LoadedMeshes[0];

        Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity()};
        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};
        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            std::array<Vector3f, 3> face_vertices;
            for (int j = 0; j < 3; j++) {
                auto vert = Vector3f(mesh.Vertices[i + j].Position.X,
                                     mesh.Vertices[i + j].Position.Y,
                                     mesh.Vertices[i + j].Position.Z) *
                            60.f;
                face_vertices[j] = vert;

                min_vert = Vector3f(std::min(min_vert.x, vert.x),
                                    std::min(min_vert.y, vert.y),
                                    std::min(min_vert.z, vert.z));
                max_vert = Vector3f(std::max(max_vert.x, vert.x),
                                    std::max(max_vert.y, vert.y),
                                    std::max(max_vert.z, vert.z));
            }

            auto new_mat =
                new Material(MaterialType::DIFFUSE_AND_GLOSSY,
                             Vector3f(0.5, 0.5, 0.5), Vector3f(0, 0, 0));
            new_mat->Kd = 0.6;
            new_mat->Ks = 0.0;
            new_mat->specularExponent = 0;

            triangles.emplace_back(face_vertices[0], face_vertices[1],
                                   face_vertices[2], new_mat);
        }

        bounding_box = Bounds3(min_vert, max_vert);

        std::vector<Object*> ptrs;
        for (auto& tri : triangles)
            ptrs.push_back(&tri);

        bvh = new BVHAccel(ptrs);
    }

    bool intersect(const Ray& ray) { return true; }

    bool intersect(const Ray& ray, float& tnear, uint32_t& index) const
    {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k) {
            const Vector3f& v0 = vertices[vertexIndex[k * 3]];
            const Vector3f& v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f& v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t,
                                     u, v) &&
                t < tnear) {
                tnear = t;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }

    Bounds3 getBounds() { return bounding_box; }

   /**
 * @brief 计算光线与三角形交点处的表面属性（法向量和纹理坐标）
 * 
 * @param P        光线与物体的交点位置（世界坐标系）
 * @param I        入射光线方向（世界坐标系，仅用于后续着色，本函数未直接使用）
 * @param index    三角形索引（标识当前交点所属的第 index 个三角形）
 * @param uv       交点在三角形内的重心坐标 (u,v)，满足 u≥0, v≥0, u+v≤1
 * @param N        [输出] 修正后的表面法向量（世界坐标系，已归一化）
 * @param st       [输出] 交点处的纹理坐标（归一化到 [0,1] 范围）
 */
void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
                          const uint32_t& index, const Vector2f& uv,
                          Vector3f& N, Vector2f& st) const
{
    // ===== 步骤1：通过三角形索引获取三个顶点的世界坐标 =====
    // vertexIndex 是全局顶点索引数组（处理OBJ等格式的顶点复用）
    // 例如：index=5 表示第5个三角形，其顶点索引为 vertexIndex[15], vertexIndex[16], vertexIndex[17]
    const Vector3f& v0 = vertices[vertexIndex[index * 3]];      // 三角形顶点0
    const Vector3f& v1 = vertices[vertexIndex[index * 3 + 1]];  // 三角形顶点1
    const Vector3f& v2 = vertices[vertexIndex[index * 3 + 2]];  // 三角形顶点2

    // ===== 步骤2：计算三角形的几何法向量（平面法向量） =====
    Vector3f e0 = normalize(v1 - v0);  // 边向量0→1（归一化）
    Vector3f e1 = normalize(v2 - v1);  // 边向量1→2（归一化）
    // 通过叉积计算垂直于三角形的法向量（右手定则：e0 × e1）
    N = normalize(crossProduct(e0, e1));  // 确保法向量单位长度（关键！避免光照计算错误）

    // ===== 步骤3：通过重心坐标插值得到交点处的纹理坐标 =====
    // 获取三角形三个顶点预定义的纹理坐标
    const Vector2f& st0 = stCoordinates[vertexIndex[index * 3]];      // 顶点0的纹理坐标
    const Vector2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];  // 顶点1的纹理坐标
    const Vector2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];  // 顶点2的纹理坐标

    // 重心坐标插值公式：st = (1-u-v)*st0 + u*st1 + v*st2
    // uv.x = u, uv.y = v（Möller–Trumbore算法输出的标准重心坐标）
    st = st0 * (1 - uv.x - uv.y) +  // 顶点0的权重 = 1-u-v
         st1 * uv.x +               // 顶点1的权重 = u
         st2 * uv.y;                // 顶点2的权重 = v

    // ===== 注意事项 =====
    // 1. 此处计算的是几何法向量（flat shading），所有交点使用同一法向量
    //    （若需平滑法向量 smooth shading，应使用顶点法向量插值，而非叉积计算）
    // 2. stCoordinates 存储的是模型UV展开后的2D坐标，与几何位置无关
    // 3. 重心坐标插值保证了纹理在三角形内线性过渡（透视校正需在光栅化阶段处理）
}

    Vector3f evalDiffuseColor(const Vector2f& st) const
    {
        float scale = 5;
        float pattern =
            (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031),
                    Vector3f(0.937, 0.937, 0.231), pattern);
    }

    Intersection getIntersection(Ray ray)
    {
        Intersection intersec;

        if (bvh) {
            intersec = bvh->Intersect(ray);
        }

        return intersec;
    }

    Bounds3 bounding_box;
    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;

    std::vector<Triangle> triangles;

    BVHAccel* bvh;

    Material* m;
};

inline bool Triangle::intersect(const Ray& ray) { return true; }
inline bool Triangle::intersect(const Ray& ray, float& tnear,
                                uint32_t& index) const
{
    return false;
}

inline Bounds3 Triangle::getBounds() { return Union(Bounds3(v0, v1), v2); }

inline Intersection Triangle::getIntersection(Ray ray)
{
    Intersection inter;

    if (dotProduct(ray.direction, normal) > 0)
        return inter;
    double u, v, t_tmp = 0;
    Vector3f pvec = crossProduct(ray.direction, e2);
    double det = dotProduct(e1, pvec);
    if (fabs(det) < EPSILON)
        return inter;

    double det_inv = 1. / det;
    Vector3f tvec = ray.origin - v0;
    u = dotProduct(tvec, pvec) * det_inv;
    if (u < 0 || u > 1)
        return inter;
    Vector3f qvec = crossProduct(tvec, e1);
    v = dotProduct(ray.direction, qvec) * det_inv;
    if (v < 0 || u + v > 1)
        return inter;
    t_tmp = dotProduct(e2, qvec) * det_inv;
    // TODO find ray triangle intersection
	if (t_tmp > EPSILON) 
	{
		inter.happened = true;
		inter.distance = t_tmp;
		inter.normal = normal;
		inter.coords = ray.origin + t_tmp * ray.direction;
		inter.m = m;
		inter.obj = this;
	}

    return inter;
}

inline Vector3f Triangle::evalDiffuseColor(const Vector2f&) const
{
    return Vector3f(0.5, 0.5, 0.5);
}
