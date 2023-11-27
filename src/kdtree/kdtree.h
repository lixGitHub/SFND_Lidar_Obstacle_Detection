/* \author Xiao Li */
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
private:
	/* data */
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

	std::vector<int> SearchHelper(std::vector<float> target, float distanceTol, int depth, std::shared_ptr<NodePoint>& node_ptr, bool debug){
		if (debug){ std::cout << "====depth: " << depth << "\n"; }
	
		std::vector<int> near_ids;

		if (nullptr == node_ptr)
		{
			if (debug){ std::cout << "node is null, return\n"; }
			return near_ids;
		}

		if (debug){  
		std::cout << "check node " << node_ptr->id << " coordinates: (";
			for (auto c: node_ptr->point)
			{
				std::cout << c << ", ";
			}
			std::cout << ")\n";
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
			if (debug){  std::cout << node_ptr->id << " is a candidate\n"; }
			float dis_square, temp_dis;
			for (int i = 0; i < target.size(); i++){
				temp_dis = node_ptr->point[i] - target[i];
				dis_square += temp_dis * temp_dis;
			}
			
			if (dis_square < distanceTol*distanceTol)
			{
				near_ids.push_back(node_ptr->id);
				if (debug){  std::cout << "find nearby: " << node_ptr->id << ",  dist: " << (dis_square) << "\n"; }
			}else{
				if (debug){  std::cout << node_ptr->id << " not near by, dist = " << dis_square << "\n"; }
			}
		}
		else {
			if (debug){  std::cout << node_ptr->id << " is not candidate.\n"; }
		}

		int dim = target.size(); // get points dimesion.
		int comp_dim = depth % dim; // calculate which dim is used to compare.
		if (debug){  std::cout << "check sub region, comp_dim = " << comp_dim <<  "\n"; }

		if (debug){  
			std::cout << "node_p coordinates: (";
			for (auto c: node_ptr->point)
			{
				std::cout << c << ", ";
			}
			std::cout << ")\n";
			std::cout << "target coordinates: (";
			for (auto c: target)
			{
				std::cout << c << ", ";
			}
			std::cout << ")\n";
			
			std::cout << "test: " <<node_ptr->point[comp_dim] << ", " <<  target[comp_dim] << ", " << distanceTol << std::endl;
		}

		// if (node_ptr->point[comp_dim] > target[comp_dim]+distanceTol)
		if (node_ptr->point[comp_dim] > target[comp_dim]-distanceTol)
		{ // only in the left region
			if (debug){  
				int left_id = node_ptr->left == nullptr ? -1 : node_ptr->left->id ;
				std::cout << "search left sid of " << node_ptr->id << " left id: " << left_id << "\n";
			}

			std::vector<int> left_nearby = SearchHelper(target, distanceTol, depth+1, node_ptr->left, debug);
			for (auto c_i: left_nearby) { near_ids.push_back(c_i); }
		}

		// else if (node_ptr->point[comp_dim] < target[comp_dim]- distanceTol)
		if (node_ptr->point[comp_dim] < target[comp_dim]+distanceTol)
		{ // only in the right region.
			if (debug){  
				int right_id = node_ptr->right == nullptr ? -1 : node_ptr->right->id ;
				std::cout << "search right sid of " << node_ptr->id << " right id: " << right_id <<"\n";
			}

			std::vector<int> right_candidate = SearchHelper(target, distanceTol, depth+1, node_ptr->right, debug);
			for (auto c_i: right_candidate) { near_ids.push_back(c_i); }
		}

		if (debug){  
			std::cout << "return: ";
			for (auto e: near_ids){
				std::cout << e << ", ";
			}
			std::cout << "\n";
		}
			
		return near_ids;
	}
	
	std::vector<int> SearchNeighbour(std::vector<float> target, float distanceTol){
		std::vector<int> ids = SearchHelper(target, distanceTol, 0, root, false);
		return ids;
	}

	void PrintTree(){
		std::queue<std::shared_ptr<NodePoint>> node_list;
		node_list.push(root);

		while (!node_list.empty())
		{
			int count = node_list.size();
			int i =0;
			while (i<count)
			{
				std::shared_ptr<NodePoint> node_ptr = node_list.front();

					// std::string node_id = node_ptr==nullptr ? "nan" : std::to_string(node_ptr->id); 
					// std::cout << node_id << "	";

				if (node_ptr == nullptr)
				{
					std::cout << "nan	";
				}
				else{
					std::cout << node_ptr->id << "	";
					node_list.push(node_ptr->left);
					node_list.push(node_ptr->right);
				}
				node_list.pop();
				i++;
			}
			std::cout << "\n";
		}
	}
};


#endif