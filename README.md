# RayTracer-Games101
在渲染中光线物体求交是性能瓶颈，本项目基于games101的框架实现了SAH-BVH加速结构，PBR材质，蒙特卡洛积分计算像素颜色，渲染模型性能提升186倍。
> **项目声明**  
> 本仓库基于 [GAMES101: Modern Computer Graphics](https://sites.cs.ucsb.edu/~lingqi/teaching/games101.html) 课程官方框架开发。
> 框架代码（`framework/` 目录）为课程提供，仅作编译依赖；
> 所有算法实现（`src/` 目录）均为本人原创。
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    if (depth > this->maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }
    Intersection intersection = Scene::intersect(ray); //获取光-物相交结构体
    Material *m = intersection.m;
    Object *hitObject = intersection.obj;
    Vector3f hitColor = this->backgroundColor;
//    float tnear = kInfinity;
    Vector2f uv;
    uint32_t index = 0;
    if(intersection.happened) {
//准备getSurface函数的参数；注意，当intersection.happened,object的求交函数被调用，index索引更新为正确三角形索引而不是橙色的0。
        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal; // normal
        Vector2f st; // st coordinates
        hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);
//        Vector3f tmp = hitPoint;//初始化csatRay输出参数，承接下面
