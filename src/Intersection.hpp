//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

struct Intersection
{
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        distance= std::numeric_limits<float>::max();
        obj =nullptr;
        m=nullptr;
		uint32_t index;  // 三角形全局索引
    }
    bool happened;
    Vector3f coords;
    Vector3f normal;
    float distance;
    Object* obj;
    Material* m;
};
#endif //RAYTRACING_INTERSECTION_H
