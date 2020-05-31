

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

template <typename PointT>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper (Node*& node, uint depth, PointT point, int id)
	{
		uint d = depth % 3;

		if (node == nullptr)
		{
			std::vector<float> vPoint (point.data, point.data+3);
			node = new Node(vPoint, id);
		}
		else if(point.data[d] < node->point[d])
		{
		/*data point is less than root insert in left child*/
		insertHelper(node->left, d+1, point, id);
		}
		else
		{
		/*data point is greater than root insert in right child*/
		insertHelper(node->right, d+1, point, id);
		}
		
	}

	void insert(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		for (uint index = 0; index < cloud->points.size(); index++)
		{
			insertHelper (root, 0, cloud->points[index], index);
		}
		// insertHelper(&root, 0, point, id);
	}

	void searchHelper (PointT target, Node*& node, int depth, float distanceTol, std::vector<int>& ids)
	{
		uint d = depth % 3;
		if (node != nullptr)
		{
			// float nodeX = node->point[0];
			// float nodeY = node->point[1];
			// float nodeZ = node->point[2];
			// float targetX = target.data[0];
			// float targetY = target.data[1];
			// float targetZ = target.data[2];

			// In the box
			// if ((nodeX >= targetX - distanceTol) && (nodeX <= targetX + distanceTol) && (nodeY >= targetY - distanceTol) && (nodeY <= targetY + distanceTol) && (nodeZ >= targetZ - distanceTol) && (node->point[2] <= targetZ + distanceTol))
			if ((node->point[0] > target.data[0] - distanceTol) && (node->point[0] < target.data[0] + distanceTol) && (node->point[1] > target.data[1] - distanceTol) && (node->point[1] < target.data[1] + distanceTol) && (node->point[2] > target.data[2] - distanceTol) && (node->point[2] < target.data[2] + distanceTol))
			{
				// float xDiff = nodeX - targetX;
				// float yDiff = nodeY - targetY;
				// float zDiff = node->point[2] - targetZ;
				// float distance = sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff);

				float distance=sqrt((node->point[0]-target.data[0])*(node->point[0]-target.data[0])+
						(node->point[1]-target.data[1])*(node->point[1]-target.data[1])+
						(node->point[2]-target.data[2])*(node->point[2]-target.data[2]));

				if (distance < distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			// Recurssively iterate
			if (target.data[d] - distanceTol < node->point[d])
			{
				searchHelper (target, node->left, d+1, distanceTol, ids);
			}
			if (target.data[d] + distanceTol > node->point[d])
			{
				searchHelper (target, node->right, d+1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper (target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




