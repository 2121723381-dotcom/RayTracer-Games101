#include <algorithm>
#include <cassert>
#include "BVH.hpp"
#include "Vector.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector<Object*>{objects[0]});
        node->right = recursiveBuild(std::vector<Object*>{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

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

//Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
//{
//	// ========== 1. 准备求交辅助数据 ==========
//	// 光线方向的倒数，用于包围盒求交时用乘法代替除法，提升速度
//	Vector3f dir_inv = ray.direction_inv;
//	// 标记光线每个轴向的方向正负：1=负方向，0=正方向
//	// 用于快速判断包围盒的两个平面哪个先被光线击中
//	std::array<int, 3> dirIsNeg{};
//	dirIsNeg[0] = ray.direction.x < 0 ? 1 : 0;
//	dirIsNeg[1] = ray.direction.y < 0 ? 1 : 0;
//	dirIsNeg[2] = ray.direction.z < 0 ? 1 : 0;
//
//	// ========== 2. 包围盒快速裁剪（核心加速逻辑）==========
//	// 先判断光线和当前BVH节点的包围盒是否相交
//	// 如果不相交，直接返回空交点，这个节点下面的所有物体都不用再检测了
//	if (!node->bounds.IntersectP(ray, dir_inv, dirIsNeg))
//	{
//		// 返回默认构造的空交点（happened=false，distance=无穷大）
//		return Intersection();
//	}
//
//	// ========== 3. 判断是否是叶子节点 ==========
//	// 叶子节点：左右孩子都为空，节点里存放了一个具体的物体
//	if (node->left == nullptr && node->right == nullptr)
//	{
//		// 【关键修改】调用物体自身的精确求交函数
//		// 多态自动处理：球体就解二次方程，网格模型就走它内部自己的BVH遍历三角形
//		// 这个函数会返回完整的交点信息（坐标、法线、材质、距离等），不需要我们手动补全
//		return node->object->getIntersection(ray);
//	}
//
//	// ========== 4. 内部节点：递归遍历左右子树 ==========
//	// 分别递归检测左、右子节点，拿到各自的最近交点结果
//	Intersection left_hit = getIntersection(node->left, ray);
//	Intersection right_hit = getIntersection(node->right, ray);
//
//	// 比较两个交点的距离，返回离相机更近的那个交点
//	// 光线追踪只需要最近的交点，更远的会被遮挡，没有意义
//	if (left_hit.distance < right_hit.distance)
//	{
//		return left_hit;
//	}
//	else
//	{
//		return right_hit;
//	}
//}