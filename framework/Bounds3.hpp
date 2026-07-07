//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum); //右边界
        pMin = Vector3f(maxNum, maxNum, maxNum); //左边界
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {} //点包围盒
    Bounds3(const Vector3f p1, const Vector3f p2) //正常包围盒
    {
		//任意两点求出对应的包围盒
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z)); //两个点 6个数 选最小三组
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z)); //两个点 6个数 选最大三组
    }

    Vector3f Diagonal() const { return pMax - pMin; }
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};


//判断与包围盒是否相交；std::array<int, 3>& dirIsNeg参数：
//dirIsNeg[0] = ray.dir.x < 0 ? 1 : 0; // x方向是否为负
//dirIsNeg[1] = ray.dir.y < 0 ? 1 : 0; // y方向是否为负
//dirIsNeg[2] = ray.dir.z < 0 ? 1 : 0; // z方向是否为负
//如果方向正 ，则先碰pMin，再碰pMax
//如果方向正 ，则先碰pMin，再碰pMax
inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
	Vector3f tr ;//tr是光线的表达式
	double t_min_x,t_max_x;
	double t_min_y, t_max_y;
	double t_min_z, t_max_z;
	if (dirIsNeg[0] == 0) 
	{
		tr.x = pMin.x;
		t_min_x = (tr.x - ray.origin.x)*invDir.x;
		tr.x = pMax.x;
		t_max_x = (tr.x - ray.origin.x)*invDir.x;
	}else if (dirIsNeg[0] == 1) 
	{
		tr.x = pMax.x;
		t_min_x = (tr.x - ray.origin.x)*invDir.x;
		tr.x = pMin.x;
		t_max_x = (tr.x - ray.origin.x)*invDir.x;
	}

	if (dirIsNeg[1] == 0)
	{
		tr.y = pMin.y;
		t_min_y = (tr.y - ray.origin.y)*invDir.y;
		tr.y = pMax.y;
		t_max_y = (tr.y - ray.origin.y)*invDir.y;
	}else if (dirIsNeg[1] == 1)
	{
		tr.y = pMax.y;
		t_min_y = (tr.y - ray.origin.y)*invDir.y;
		tr.y = pMin.y;
		t_max_y = (tr.y - ray.origin.y)*invDir.y;
	}

	if (dirIsNeg[2] == 0)
	{
		tr.z = pMin.z;
		t_min_z = (tr.z - ray.origin.z)*invDir.z;
		tr.z = pMax.z;
		t_max_z = (tr.z - ray.origin.z)*invDir.z;
	}else if (dirIsNeg[2] == 1)
	{
		tr.z = pMax.z;
		t_min_z = (tr.z - ray.origin.z)*invDir.z;
		tr.z = pMin.z;
		t_max_z = (tr.z - ray.origin.z)*invDir.z;
	}
	//只有光线进入了三个平面，才算进入包围盒
	double t_enter = std::max(t_min_x, std::max(t_min_y, t_min_z));
	//只要光线离开了一个平面，就算离开包围盒
	double t_exit = std::min(t_max_x, std::min(t_max_y, t_max_z)); // min 同理
	//当且仅当进入时间小于出射时间，且出射时间是有效值才返回，进入时间可以<0是因为，光源可以在包围盒内。
	if (t_enter < t_exit && t_exit > 0)return true;
	else return false;
}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
