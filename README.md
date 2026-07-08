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

getSurfaceProperties函数

注：这个函数的参数分两种：一种是只读参数，传入函数内部，带const；一种是输出参数，传入的初始值最后被函数修改，输出函数，带‘&’

输入参数：p：光-物相交点；I：光线方向；index：索引；uv：光-物相交结构体对应的纹理坐标

逻辑：（1）根据传进的索引访问三角形顶点，构建向量叉乘得到输出参数N；
（2）算出三角形三个顶点对应纹理坐标st0，st1，st2，利用Möller-Trumbore算法算出的u，v插值计算出光物相交点对应的纹理坐标st.

输出参数：三角形的法线N，碰撞点对应的纹理坐标st.
```cpp
/**
 * 计算光线与三角形交点处的表面属性（法向量和纹理坐标）
 *  P        光线与物体的交点位置（世界坐标系）
 *  I        入射光线方向（世界坐标系，仅用于后续着色，本函数未直接使用）
 *  index    三角形索引（标识当前交点所属的第 index 个三角形）
 *  uv       交点在三角形内的重心坐标 (u,v)，满足 u≥0, v≥0, u+v≤1
 *  N        [输出] 修正后的表面法向量（世界坐标系，已归一化）
 *  st       [输出] 交点处的纹理坐标（归一化到 [0,1] 范围）
 */
void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
                          const uint32_t& index, const Vector2f& uv,
                          Vector3f& N, Vector2f& st) const
{
    // ===== 步骤1：通过三角形索引获取三个顶点的世界坐标 =====
    // vertexIndex 是全局顶点索引数组（OBJ等格式的顶点）
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
```

输入：指向物体的指针数组，叶子节点储存最大物体数量，分割方式

逻辑：判断指针是否为空->调用底层bvh构建函数recursiveBuild(primitives)，构建之前计时间，构建完毕后停止计时

输出：无 但是加速结构已经搭建完毕
```cpp
BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p)) 
// maxPrimsInNode：叶子节点允许的最大Object数量，一般设置为1，这样一个叶子节点只需要做一次物体-光线求交
// splitMethod：决定如何将复合节点内地的物体划分到左右子树中的方法；
// 程序一般枚举{ Native（朴素法），SAH（表面面积启发式）}
// primitives：参数传入p指针，把指向三角形的指针数组通过move函数移动到primitives，三角形群的指针交给BVHAccel对象
{
    time_t start, stop;
    time(&start); // 构建开始前，计算当前时间
//没有物体则不构建加速结构
    if (primitives.empty())
        return;
//有物体则构建加速结构根节点
    root = recursiveBuild(primitives);

    time(&stop); // 构建完毕后，计算当前时间
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}
```

输入：物体指针群

逻辑：通过传进的物体指针的个数，分3种情况构建包围盒，最后返回指向构建好的bvh节点的指针；

objects.size() == 1：节点是叶子节点，node左右指针为空，物体指向

objects.size() == 2：节点简单二分，将object参数改成两个物体，分别递归调用recursiveBuild()，最后用Union函数封装成一个大包围盒

objects.size() >2：计算最分散的轴->将指针索引顺序与物体从左到右排序->确定中间索引然后分开一半 左边是leftshape，右边是rightshape->

将leftshape与rightshape作为recursiveBuild新参数，递归构建子树，并分别用node->left，node->right 指向recursiveBuild(leftshapes),recursiveBuild(rightshapes)

输出：指向构造好的加速node节点的指针
```cpp
//（1）：初始化node指针，BVHBuildNode* node = new BVHBuildNode(); node指向下面结构体；
BVHBuildNode(){
        bounds = Bounds3();
        left = nullptr;right = nullptr;
        object = nullptr;
    }
};
//（2）：遍历所有物体列表，用一个最小包围盒装上所有物体。
Bounds3 bounds;
for (int i = 0; i < objects.size(); ++i)
 bounds = Union(bounds, objects[i]->getBounds());
//（3）：根据物体数量 N 执行不同的构建策略
N=1，节点是叶子节点。
if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0]; 
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
//N=2，节点简单二分，将object参数改成两个物体，分别递归调用recursiveBuild()，最后用Union函数封装成一个大包围盒
else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector<Object*>{objects[0]});
        node->right = recursiveBuild(std::vector<Object*>{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
//N>2，
Bounds3 centroidBounds;
for (int i = 0; i < objects.size(); ++i)
//遍历物体列表，通过物体包围盒的质心（空间中心点），计算物体群在xyz轴最分散的轴
//注意：用质心计算而不用物体包围盒本身进行划分，原因如下：
// 1. 避免大物体主导空间划分：若直接使用包围盒范围（如Union(objects[i]->getBounds())），
// 超大物体（如场景地面）会强制将所有物体划分到同一子树，导致树极度不平衡。
// 2. 质心仅反映位置分布：质心坐标仅表示物体在空间中的"重心位置"，与物体实际尺寸解耦，
// 能真实反映物体群的空间分布密度（例如：密集的小物体群仍会被识别为紧凑区域）。
// 3. 防止空间扭曲：当场景同时存在极大物体（如天空盒）和极小物体（如粒子）时，
// 包围盒划分会因极大物体的边界主导而扭曲空间分割，而质心划分能保持子空间体积合理。
// 4. 平衡查询效率：以质心分布最广的轴进行中位数分割，可使左右子树覆盖的空间体积更均衡，
// 显著减少光线遍历时的无效包围盒重叠检测（对比直接用包围盒划分，性能提升可达30%+）。
    centroidBounds =
        Union(centroidBounds, objects[i]->getBounds().Centroid()); 
//dim记录哪个轴最分散，x=0，y=1，z=2；
int dim = centroidBounds.maxExtent(); 
// 确定轴后，此时，物体指针指的物体的位置并不是按照顺序的，可能是：
objects[0] = plane;  // 指向0x2000（地面，x=100.0）
objects[1] = ball;   // 指向0x1000（球体，x=1.0）
objects[2] = cube;   // 指向0x3000（立方体，x=2.5）
// 排好才能确定中点，sort就是快速排序方法。
// sort(范围起点，范围中点，比较规则)，当符合返回规则时，f1必须排在f2前面。
// 当y轴，z轴最长时，改变规则即可
std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
```
