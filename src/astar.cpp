#include "../include/astar/astar.h"

typedef pair<float, int> pi;

static const float XY_RESOLUTION = 0.05; // [m] Grid resolution of the map
static const float STEP = 1.0; // Path Resolution


// Publisher Variables
ros::Publisher start_pose_pub;
ros::Publisher goal_pose_pub;
ros::Publisher astar_path_pub;

// Variables for start and goal points
float sx;
float sy;
float gx;
float gy;

geometry_msgs::PointStamped start_point; // Start pose msg
bool valid_start; // Start pose validity check
geometry_msgs::PointStamped goal_point; // Goal pose msg
bool valid_goal; // Goal pose validity check

nav_msgs::OccupancyGrid::Ptr grid; // Pointer to the occupancy grid msg
int grid_height;
int grid_width;
bool** bin_map; // 2D Binary map of the grid 
int** acc_obs_map;

nav_msgs::Path path; // Astar Path


int calc_index(Node2D n) {
	return ((int)(n.get_y() * grid_width + n.get_x())); // Calculating node index
}


float calc_heuristic_cost(float x, float y, float gx, float gy) {
	return hypot(x - gx, y - gy);
}


bool check_collision(Node2D n, bool** bin_map) {

	if(bin_map[(int)n.get_x()][(int)n.get_y()]) {
		// ROS_INFO("IN COLLISION!");
		return true;
	}

	// ROS_INFO("NO COLLISION!");
	return false; // NO collision
}


int astar(float sx, float sy, float gx, float gy) {

	nav_msgs::Path astar_path;
	astar_path.header.stamp = ros::Time::now();
	astar_path.header.frame_id = "/map";

	geometry_msgs::PoseStamped ps;
	ps.header.stamp = ros::Time::now();
	ps.header.frame_id = "/map";

	sx = round(sx/XY_RESOLUTION);
	sy = round(sy/XY_RESOLUTION);
	Node2D start_node = Node2D(sx, sy, 0, NULL);

	gx = round(gx/XY_RESOLUTION);
	gy = round(gy/XY_RESOLUTION);
	Node2D goal_node = Node2D(gx, gy, 0, NULL);

	// std::vector<std::vector<float>> motions = { {0.05, 0.0, 0.05}, {-0.05, 0.0, 0.05}, {0.0, 0.05, 0.05}, {0.0, -0.05, 0.05}, {0.05, 0.05, sqrt(0.1)}, {-0.05, -0.05, sqrt(0.1)}, {-0.05, 0.05, sqrt(0.1)}, {0.05, -0.05, sqrt(0.1)}}; // x and y motion inputs for child nodes
	std::vector<std::vector<float>> motions = { {STEP, 0.0, STEP}, {-STEP, 0.0, STEP}, {0.0, STEP, STEP}, {0.0, -STEP, STEP}, {STEP, STEP, sqrt(STEP*STEP*2)}, {-STEP, -STEP, sqrt(STEP*STEP*2)}, {-STEP, STEP, sqrt(STEP*STEP*2)}, {STEP, -STEP, sqrt(STEP*STEP*2)}}; // x and y motion inputs for child nodes
	// std::vector<std::vector<float>> motions = { {STEP, 0.0, STEP}, {-STEP, 0.0, STEP}, {0.0, STEP, STEP}, {0.0, -STEP, STEP}, {STEP, STEP, STEP}, {-STEP, -STEP, STEP}, {-STEP, STEP, STEP}, {STEP, -STEP, STEP}};

	Node2D current_node;
	Node2D new_node;
	float node_cost;
	float cost_so_far = 0;
	pair<float, int> current_ind;
	int new_ind;

	std::map<int, Node2D> open_list;
	std::map<int, Node2D> closed_list;

	open_list[calc_index(start_node)] = start_node;
	// cout << "Open List - " << endl;
	// display_map(open_list);

	priority_queue<pi, vector<pi>, greater<pi>> pq;
	pq.push(make_pair(0, calc_index(start_node)));
	// display_pq(pq);

	while(true) {

		// cin.get();

		if(open_list.empty()) {
			ROS_INFO("SOLUTION DOESN'T EXIST - NO NODES FOUND IN OPEN LIST");
			break;
		}

		// display_pq(pq);

		current_ind = pq.top();
		pq.pop();

		current_node = open_list[current_ind.second];
		// cost_so_far = cost_so_far + current_node.get_cost();
		closed_list[current_ind.second] = current_node;
		open_list.erase(current_ind.second);
		// cout << "Open List - " << endl;
		// display_map(open_list);

		if(hypot(current_node.get_x() - gx, current_node.get_y() - gy) <= 1.0) {
			// ROS_INFO("ASTAR PATH FOUND");
			break;
		}
		
		for (int i = 0; i < motions.size(); ++i) {	
			// cin.get();

			node_cost = calc_heuristic_cost(current_node.get_x() + motions[i][0], current_node.get_y() + motions[i][1], gx, gy);
			node_cost = node_cost + motions[i][2] + current_node.get_g_cost();
			new_node = Node2D(current_node.get_x() + motions[i][0], current_node.get_y() + motions[i][1], node_cost, current_ind.second);
			new_node.set_g_cost(motions[i][2] + current_node.get_g_cost());

			new_ind = calc_index(new_node);

			if(check_collision(new_node, bin_map)) {
				continue;
			}

			if(closed_list.count(new_ind)) {
				continue;
			}

			if(!open_list.count(new_ind)) {
				open_list[new_ind] = new_node;
				pq.push(make_pair(new_node.get_cost(), new_ind));
			} else {
				if(open_list[new_ind].get_cost() > new_node.get_cost()) {
					open_list[new_ind] = new_node;
				}
			}
		}
	}


	while(current_node.get_pind() != NULL) {
		ps.pose.position.x = current_node.get_x() * XY_RESOLUTION;
		ps.pose.position.y = current_node.get_y() * XY_RESOLUTION;
		astar_path.poses.push_back(ps);
		current_node = closed_list[current_node.get_pind()];
	}
	astar_path_pub.publish(astar_path);

	return astar_path.poses.size() * XY_RESOLUTION;
}


/*
	Subcribes/callback: /initial_pose
	Publishes: /start_pose

	Callback function to retrieve the initial pose and display it in rviz
*/
void callback_start_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {

	start_point.header.stamp = ros::Time::now();
	start_point.header.frame_id = "map";
	start_point.point = pose->pose.pose.position;

	start_pose_pub.publish(start_point);

	sx = start_point.point.x;
	sy = start_point.point.y;

	path.poses.clear();

	astar(sx, sy, gx, gy);
}


/*
	Subcribes/callback: /move_base_simple/goal
	Publishes: /goal_pose

	Callback function to retrieve the final pose and display it in rviz
*/
void callback_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& pose) {

	goal_point.header.stamp = ros::Time::now();
	goal_point.header.frame_id = "map";
	goal_point.point = pose->pose.position;

	goal_pose_pub.publish(goal_point);

	gx = goal_point.point.x;
	gy = goal_point.point.y;
	
	path.poses.clear();

	astar(sx, sy, gx, gy);
}


/*
	Subscribes/Callback: /map
	Publishes: None

	Callback function to retrieve the occupancy grid and construct a 2D binary obstacle map 
*/
void callback_map(const nav_msgs::OccupancyGrid::Ptr map) {

	grid = map;
	ROS_INFO("Recieved the occupancy grid map");

	grid_height = map->info.height;
	grid_width = map->info.width;
	bin_map = new bool*[grid_width];

	for (int x = 0; x < grid_width; ++x) { bin_map[x] = new bool[grid_height]; }

	for (int x = 0; x < grid_width; ++x) {
		for (int y = 0; y < grid_height; ++y) {
			bin_map[x][y] = map->data[y * grid_width + x] ? true : false;
		}
	}

	acc_obs_map = new int* [grid_width];

	for (int x = 0; x < grid_width; x++) {
		acc_obs_map[x] = new int[grid_height];
		for (int y = 0; y < grid_height; y++) {
			acc_obs_map[x][y] = (bin_map[x][y] > 0);
		}
	}

	for (int x = 0; x < grid_width; x++) {
		for (int y = 1; y < grid_height; y++) {
			acc_obs_map[x][y] = acc_obs_map[x][y-1] + acc_obs_map[x][y];
		}
	}

	for (int y = 0; y < grid_height; y++) {
		for (int x = 1; x < grid_width; x++) {
			acc_obs_map[x][y] = acc_obs_map[x-1][y] + acc_obs_map[x][y];
		}
	}
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "astar");

	ros::NodeHandle nh;

	// Publishers
	start_pose_pub = nh.advertise<geometry_msgs::PointStamped>("start_pose", 1000);
	goal_pose_pub = nh.advertise<geometry_msgs::PointStamped>("goal_pose", 1000);
	astar_path_pub = nh.advertise<nav_msgs::Path>("astar_path", 1000);

	// Subscribers
	ros::Subscriber start_pose_sub = nh.subscribe("initialpose", 1000, callback_start_pose);
	ros::Subscriber goal_pose_sub = nh.subscribe("move_base_simple/goal", 1000, callback_goal_pose);
	ros::Subscriber map_sub = nh.subscribe("map", 1, callback_map);

	ros::spin();

	return 0;
}