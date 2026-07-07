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
