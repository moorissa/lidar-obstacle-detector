/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};


struct KdTree
{
	Node* root;
	int idx;
	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	// either use return or use a double pointer/pointer by reference. 
	// using the latter methods, the changes are directly reflected in the calling function
	void insertRec(Node** root, const std::vector<float>& point, int depth, int id)
	{
		if (*root == NULL)
		{
			*root = new Node(point, id);
		}
		else
		{
			int idx = depth % 3;  // Cycles through dimensions 0, 1, 2 (x, y, z)
			if (point[idx] < ((*root)->point[idx]))
				insertRec(&((*root)->left), point, depth + 1, id);
			else
				insertRec(&((*root)->right), point, depth + 1, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertRec(&root, point, 0, id);
	}

	// return a list of point ids in the tree that are within distance of target
	bool distCompare(const std::vector<float>& point1, const std::vector<float>& point2, float distTol)
	{
		// Ensure both points have at least 3 dimensions
		if (point1.size() < 3 || point2.size() < 3) {
			return false;
		}
		
		float x = point1[0] - point2[0];
		float y = point1[1] - point2[1];
		float z = point1[2] - point2[2];
		
		float distSquared = x * x + y * y + z * z;
		return distSquared <= (distTol * distTol);
	}

	void searchRec(const std::vector<float>& target, Node* root, int depth, float distTol, std::vector<int>& ids)
	{
		if (root != NULL)
		{	
			// 3D bounding box check
			if((root->point[0] >= (target[0] - distTol) && root->point[0] <= (target[0] + distTol)) && 
			(root->point[1] >= (target[1] - distTol) && root->point[1] <= (target[1] + distTol)) &&
			(root->point[2] >= (target[2] - distTol) && root->point[2] <= (target[2] + distTol))) {
				
				if (distCompare(target, root->point, distTol))
				{
					ids.push_back(root->id);
				}
			}

			int idx = depth % 3;  // Cycles through x, y, z dimensions
			
			if ((target[idx] - distTol) < root->point[idx])
				searchRec(target, root->left, depth + 1, distTol, ids);

			if ((target[idx] + distTol) > root->point[idx])
				searchRec(target, root->right, depth + 1, distTol, ids);
		}
	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchRec(target, root, 0, distanceTol, ids);
		return ids;
	}
};


// Declare clustering function
std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>>& points,
    KdTree* tree,
    float distanceTol);

#endif // KDTREE_H




