/* author Xiao Li */
// 3D kd tree

#ifndef MYKDTREE
#define MYKDTREE

#include <cmath>
#include <queue>
#include <memory>
#include <vector>

// Structure to represent node of kd tree

class NodePoint
{
public:
	int id;
	std::vector<float> point;

	std::shared_ptr<NodePoint> left;
	std::shared_ptr<NodePoint> right;

	NodePoint(std::vector<float> ipoint, int setid) : point(ipoint), id(setid), left(NULL), right(NULL) {}
	~NodePoint(){}
};


class KdTree3D
{
private:
	std::shared_ptr<NodePoint> root;
public:
	KdTree3D(){}
	~KdTree3D(){}

	void InsertPoint(std::vector<float> point, int setid){
		std::shared_ptr<NodePoint> current_node = root;
		int comp_dim = 0;
		int dim = point.size();
		if (nullptr == root)
		{
			// root = new Node(point, id);
			root = std::make_shared<NodePoint>(point, setid);
		}
		else
		{		
			while (nullptr != current_node)
			{
				if (point[comp_dim] >= current_node->point[comp_dim])
				{
					if (nullptr == current_node->right)
					{
						current_node->right = std::make_shared<NodePoint>(point, setid);
						break;
					}
					else
					{
						current_node = current_node->right;
					}
				}
				else
				{
					if (nullptr == current_node->left)
					{
						current_node->left = std::make_shared<NodePoint>(point, setid);
						break;
					}
					else
					{
						current_node = current_node->left;
					}
				}
				comp_dim = (++comp_dim)%dim;
			}
		}
	}

	std::vector<int> SearchHelper(std::vector<float> target, float distanceTol, int depth, std::shared_ptr<NodePoint>& node_ptr){
		
		std::vector<int> near_ids;
		if (nullptr == node_ptr)
		{
			return near_ids;
		}
			
		// check if the current node is a candidate.
		bool is_candidate = true;
		for (int i = 0; i < target.size(); i++)
		{
			if (abs(node_ptr->point[i] - target[i]) > distanceTol)
			{
				is_candidate = false;
				break;
			}
		}
		if (is_candidate)
		{
			float dist_square, temp_dist;
			for (int i = 0; i < target.size(); i++){
				temp_dist = node_ptr->point[i] - target[i];
				dist_square += temp_dist * temp_dist;
			}
			
			if (dist_square < distanceTol*distanceTol)
			{
				near_ids.push_back(node_ptr->id);
			}
		}

		int dim = target.size(); // get points dimesion.
		int comp_dim = depth % dim; // calculate which dim is used to compare.

		if (node_ptr->point[comp_dim] > target[comp_dim]-distanceTol)
		{ // check the left region
			std::vector<int> left_nearby = SearchHelper(target, distanceTol, depth+1, node_ptr->left);
			for (auto c_i: left_nearby) { near_ids.push_back(c_i); }
		}

		if (node_ptr->point[comp_dim] < target[comp_dim]+distanceTol)
		{ // check the right region.
			std::vector<int> right_candidate = SearchHelper(target, distanceTol, depth+1, node_ptr->right);
			for (auto c_i: right_candidate) { near_ids.push_back(c_i); }
		}
			
		return near_ids;
	}
	
	std::vector<int> SearchNeighbour(std::vector<float> target, float distanceTol){
		std::vector<int> ids = SearchHelper(target, distanceTol, 0, root);
		return ids;
	}

};


#endif