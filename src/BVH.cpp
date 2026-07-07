#include <algorithm>
#include <cassert>
#include "BVH.hpp"
#include "Vector.hpp"

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

//标题：recursiveBuild(std::vector<Object*> objects)//参数：物体指针群
//核心逻辑：通过传进的物体指针的个数，分3种情况构建包围盒，最后返回指向构建好的bvh节点的指针；
//基本步骤：
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


Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
	Vector3f dir_inv = ray.direction_inv;
	std::array<int, 3> dirIsNeg{};
	dirIsNeg[0] = ray.direction.x < 0 ? 1 : 0; // x方向是否为负
    dirIsNeg[1] = ray.direction.y < 0 ? 1 : 0; // y方向是否为负
    dirIsNeg[2] = ray.direction.z < 0 ? 1 : 0; // z方向是否为负
	Intersection inter;
	if (!node->bounds.IntersectP(ray, dir_inv, dirIsNeg)) return inter;
	//以下情况都是已经打到包围盒的情况
	//叶子节点情况
	if (node->left == nullptr && node->right == nullptr) return node->object->getIntersection(ray);
	//复合节点情况
	if (node->left != nullptr && node->right != nullptr)
	{
		// 正确写法
		Intersection left_hit = getIntersection(node->left, ray);
		Intersection right_hit = getIntersection(node->right, ray);
		// 返回距离更近的交点
		return left_hit.distance < right_hit.distance ? left_hit : right_hit;
	}
	else return inter;
}
