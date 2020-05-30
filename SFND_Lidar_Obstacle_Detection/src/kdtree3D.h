

// Structure to represent node of kd tree
struct Node
{
	// std::vector<float> point;
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper (Node** node, uint depth, pcl::PointXYZI point, int id)
	{
		if (*node == nullptr)
		{
			*node = new Node(point, id);
		}
		else
		{
			uint d = depth % 3;

			if (d == 0)
			{
				if (point.x < (*node)->point.x)
				{
					insertHelper(&(*node)->left, depth+1, point, id);
				}
				else
				{
					insertHelper(&(*node)->right, depth+1, point, id);
				}
			}
			else if (d == 1)
			{
				if (point.y < (*node)->point.y)
				{
					insertHelper(&(*node)->left, depth+1, point, id);
				}
				else
				{
					insertHelper(&(*node)->right, depth+1, point, id);
				}
			}
			else
			{
				if (point.z < (*node)->point.z)
				{
					insertHelper(&(*node)->left, depth+1, point, id);
				}
				else
				{
					insertHelper(&(*node)->right, depth+1, point, id);
				}
			}
			
		}
		
	}

	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		
		insertHelper(&root, 0, point, id);
	}

	void searchHelper (pcl::PointXYZI target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != nullptr)
		{
			float nodeX = node->point.x;
			float nodeY = node->point.y;
			float nodeZ = node->point.z;
			float targetX = target.x;
			float targetY = target.y;
			float targetZ = target.z;

			// In the box
			if ((nodeX >= targetX - distanceTol) && (nodeX <= targetX + distanceTol) && (nodeY >= targetY - distanceTol) && (nodeY <= targetY + distanceTol) && (nodeZ >= targetZ - distanceTol) && (nodeZ <= targetZ + distanceTol))
			{
				float xDiff = nodeX - targetX;
				float yDiff = nodeY - targetY;
				float zDiff = nodeZ - targetZ;
				float distance = sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff);

				if (distance < distanceTol)
					ids.push_back(node->id);
			}

			// Recurssively iterate
			if (depth % 3 == 0)  // X-axis
			{
				if (targetX - distanceTol < nodeX)
				{
					searchHelper (target, node->left, depth+1, distanceTol, ids);
				}
				if (targetX + distanceTol > nodeX)
				{
					searchHelper (target, node->right, depth+1, distanceTol, ids);
				}
			}
			else if (depth % 3 == 1) // Y-axis
			{
				if (targetY - distanceTol < nodeY)
				{
					searchHelper (target, node->left, depth+1, distanceTol, ids);
				}
				if (targetY + distanceTol > nodeY)
				{
					searchHelper (target, node->right, depth+1, distanceTol, ids);
				}
			}
			else // Z-axis
			{
				if (targetZ - distanceTol < nodeZ)
				{
					searchHelper (target, node->left, depth+1, distanceTol, ids);
				}
				if (targetZ + distanceTol > nodeZ)
				{
					searchHelper (target, node->right, depth+1, distanceTol, ids);
				}
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper (target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




