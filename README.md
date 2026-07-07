# RayTracer-Games101
在渲染中光线物体求交是性能瓶颈，本项目基于games101的框架实现了SAH-BVH加速结构，PBR材质，蒙特卡洛积分计算像素颜色，渲染模型性能提升186倍。
> **项目声明**  
> 本仓库基于 [GAMES101: Modern Computer Graphics](https://sites.cs.ucsb.edu/~lingqi/teaching/games101.html) 课程官方框架开发。
> 框架代码（`framework/` 目录）为课程提供，仅作编译依赖；
> 所有算法实现（`src/` 目录）均为本人原创。
### 关键代码片段
输入：采样光线，当前光线反射次数；

逻辑：传进的光线与物体求交，如果相交，获取材质，法线等信息，根据材质分三种情况计算，最后光线与漫反射材质相交用phong模型计算；

输出：每个采样光线计算得到得颜色 hitcolor.
```cpp
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    if (depth > this->maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }
    Intersection intersection = Scene::intersect(ray); //获取光-物相交结构体
    Material *m = intersection.m;
    Object *hitObject = intersection.obj; //光-物相交的物体获得调用Object类函数的能力
    Vector3f hitColor = this->backgroundColor;
    Vector2f uv;
    uint32_t index = 0;
    if(intersection.happened) {
//准备getSurface函数的参数；注意，当intersection.happened,object的求交函数被调用，index索引更新为正确三角形索引而不是橙色的0。
//且下面的hitPoint，N，st都更新为相交物体的点，法线，纹理坐标。
        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal; // normal
        Vector2f st; // st coordinates
//这个函数通过上面求交得到的具体hitPoint, ray.direction, index，uv去改变，N，st
        hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);
}
```

情况1：反射与折射材质，玻璃

通过上面逻辑对已知的表面法线，光线方向，用折射函数，反射函数计算出折射方向，反射方向

将新光线起点向法线方向偏移防止自己相交，最后递归调用castRay函数，返回折射光线，反射光线的颜色值
```cpp
case REFLECTION_AND_REFRACTION: //玻璃材质：反射加折射
            {
                Vector3f reflectionDirection = normalize(reflect(dir, N)); //反射方向
                Vector3f refractionDirection = normalize(refract(dir, N, payload->hit_obj->ior)); //折射后出射方向，ior：相对折射率。
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? 
                                             hitPoint - N * scene.epsilon :
                                             hitPoint + N * scene.epsilon; //反射光线起点，scene.epsilon是>0的微小量，防止光线自相交
                Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                                             hitPoint - N * scene.epsilon :
                                             hitPoint + N * scene.epsilon; //折射光线起点
                Vector3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1); //递归一次，深度+1
                Vector3f refractionColor = castRay(refractionRayOrig, refractionDirection, scene, depth + 1);//直到打在漫反射材质上，返回颜色值
                break;
                float kr = fresnel(dir, N, payload->hit_obj->ior);
                hitColor = reflectionColor * kr + refractionColor * (1 - kr);
                break;
            }
```

最后计算总颜色值涉及到了菲涅尔系数的计算（不同角度下，光线反射的量不同），下面给出源码

输入：入射方向，法线，物质折射率，要更改的菲涅尔系数

逻辑：根据原理公式计算出在当前角度，反射光线的量

输出：更改之后的Kr
```cpp
float fresnel(const Vector3f &I, const Vector3f &N, const float &ior)
{
    float cosi = clamp(-1, 1, dotProduct(I, N)); //入射方向和法线的余弦，clamp函数防止浮点误差，cos值不落在（-1，1）区间
    float etai = 1, etat = ior; //etai默认空气折射率，etat物质折射率
//因为折射包括从内到外，从外到内射入，都需要计算反射与折射的量；法线是在表面向外的，如果cos>0,就是从内往外射出，则需要交换etai，etat。
    if (cosi > 0) {  std::swap(etai, etat); } 
    // Compute sini using Snell's law
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi)); //计算折射光线的sin值
    // Total internal reflection
    if (sint >= 1) {
        return 1; //发生全反射，折射量为0，菲涅尔系数返回1
    }
//折射情况成立，根据公式构造计算Kr
    else {
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        cosi = fabsf(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) / 2; 
    }
    // As a consequence of the conservation of energy, transmittance is given by:
    // kt = 1 - kr;
}
```

情况2：全反射，镜子

输入：m-type 材质类型

逻辑：
1：计算反射量菲涅尔系数Kr；
2：调用reflect计算反射方向并更新反射光线起点reflectionRayOrig；
3：用新起点新方向递归调用castRay并乘该光线对应的占比Kr；
输出：hitColor = castRay(Ray(reflectionRayOrig, reflectionDirection),depth + 1) * kr;
```cpp
 case REFLECTION:
            {
                float kr = fresnel(dir, N, payload->hit_obj->ior);
                Vector3f reflectionDirection = reflect(dir, N); //传入入射方向，法线，计算反射光线方向
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint + N * scene.epsilon :
                                             hitPoint - N * scene.epsilon;  //撞击点+防自碰偏移点，构成反射光线起点
                hitColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1) * kr; //递归计算反射光线，直到打在漫反射材质上，返回颜色值
                break;
            }
```
Diffuse Material 漫反射材质：木头

输入：无，不是REFLECTION_AND_REFRACTION，REFLECTION就是漫反射

逻辑：1：预处理阴影检测起点，避免自阴影

2（1）：区分面光源，点光源，当前面光源不考虑

2（2）：点光源：计算光源方向与距离->直接计算漫反射系数->执行阴影检测->用 (1-inShadow) 修正光照贡献->合成最终颜色

输出：hitColor = lightAmt * (hitObject->evalDiffuseColor(st) * m->Kd + specularColor * m->Ks)
```cpp
default:
            {
              //计算漫反射材质颜色前，已知知道了着色点的法线以及位置，由于浮点数的误差，需要在计算颜色之前让着色点位置向法线方向挪动，避免计算阴影时，起点发射光线被物体自身遮挡，本身发光物体被判定为伪阴影
                Vector3f lightAmt = 0, specularColor = 0;
                Vector3f shadowPointOrig = (dotProduct(ray.direction, N) < 0) ?
                                           hitPoint + N * EPSILON :
                                           hitPoint - N * EPSILON;
                // [comment]
                // Loop over all lights in the scene and sum their contribution up
                // We also apply the lambert cosine law
                // [/comment]
                for (uint32_t i = 0; i < get_lights().size(); ++i)
                {
                    auto area_ptr = dynamic_cast<AreaLight*>(this->get_lights()[i].get());
                    if (area_ptr)
                    {
                        // Do nothing for this assignment
                    }
                    else
                    {
                        Vector3f lightDir = get_lights()[i]->position - hitPoint; //光源到着色点的向量
                        // square of the distance between hitPoint and the light
                        float lightDistance2 = dotProduct(lightDir, lightDir);
                        lightDir = normalize(lightDir); //灯光方向
                        float LdotN = std::max(0.f, dotProduct(lightDir, N));//光线与法线的点乘
// 下面是阴影检测的逻辑，当且仅当在漫反射材质上做阴影检测，为什么只在漫反射材质做阴影检测？：
//因为当得到折射，反射材质，都会递归castRay()，最后遇到漫反射材质用phong模型作色 才进行阴影检测
                        Object *shadowHitObject = nullptr;
                        float tNearShadow = kInfinity;
//如果构建了bvh加速结构，传入检测点，方向参数就能得到是否与物体相交，相交返回true，返回1；不相交返回false，返回0；
                        bool inShadow = bvh->Intersect(Ray(shadowPointOrig, lightDir)).happened; 
//计算漫反射项phong：kd*intensity(I*N)
                        lightAmt += (1 - inShadow) * get_lights()[i]->intensity * LdotN; 
                        Vector3f reflectionDirection = reflect(-lightDir, N);
                        specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, ray.direction)),
                                              m->specularExponent) * get_lights()[i]->intensity;
                    }
                }
//遵循Phong模型的最终光照计算 L= Ld + Ls
                hitColor = lightAmt * (hitObject->evalDiffuseColor(st) * m->Kd + specularColor * m->Ks);
                break;
            }

```
