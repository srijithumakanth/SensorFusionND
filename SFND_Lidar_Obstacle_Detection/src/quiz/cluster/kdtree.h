/* \author Aaron Brown */
// Quiz on implementing kd tree

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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper (Node** node, uint depth, std::vector<float> point, int id)
	{
		if (*node == nullptr)
		{
			*node = new Node(point, id);
		}
		else
		{
			uint d = depth % 2;
			if (point[d] < (*node)->point[d])
			{
				insertHelper(&(*node)->left, depth+1, point, id);
			}
			else
			{
				insertHelper(&(*node)->right, depth+1, point, id);
			}
			 
		}
		
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		
		insertHelper(&root, 0, point, id);
	}

	void searchHelper (std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != nullptr)
		{
			float nodeX = node->point[0];
			float nodeY = node->point[1];
			float targetX = target[0];
			float targetY = target[1];

			// In the box
			if ((nodeX >= targetX - distanceTol) && (nodeX <= targetX + distanceTol) && (nodeY >= targetY - distanceTol) && (nodeY <= targetY + distanceTol))
			{
				float xDiff = nodeX - targetX;
				float yDiff = nodeY - targetY;
				float distance = sqrt(xDiff * xDiff + yDiff * yDiff);

				if (distance < distanceTol)
					ids.push_back(node->id);
			}

			// Recurssively iterate
			int d = depth % 2;
			if (target[d] - distanceTol < node->point[d])
			{
				searchHelper (target, node->left, depth+1, distanceTol, ids);
			}
			
			if (target[d] + distanceTol > node->point[d])
			{
				searchHelper (target, node->right, depth+1, distanceTol, ids);
			}

		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper (target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




