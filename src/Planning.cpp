#include "Planning.hpp"

PlanningNode::PlanningNode() : rclcpp::Node("planning_node")
{
  // Client for map
  map_client_ = create_client<nav_msgs::srv::GetMap>("/map_server/map");

  // Service for path
  plan_service_ = create_service<nav_msgs::srv::GetPlan>(
    "plan_path",
    std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2));

  // Publisher for path
  // add code here

  RCLCPP_INFO(get_logger(), "Planning node started.");

  // Connect to map server
  while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_logger(), "Waiting for map server...");
  }

  // Request map
  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  auto future = map_client_->async_send_request(
    request, std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Trying to fetch map...");
}

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future)
{
  auto response = future.get();
  if (response) {
    map_ = response->map;
    RCLCPP_INFO(
      get_logger(), "Map received: %d x %d cells, resolution: %.3f m/cell", map_.info.width,
      map_.info.height, map_.info.resolution);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to receive map.");
  }
}

void PlanningNode::planPath(
  const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
  std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
{
  // add code here

  // ********
  // * Help *
  // ********
  /*
    aStar(request->start, request->goal);
    smoothPath();

    path_pub_->publish(path_);
    */
}

void PlanningNode::dilateMap()
{
  nav_msgs::msg::OccupancyGrid dilatedMap = map_;

  map_ = dilatedMap;
}

void PlanningNode::aStar(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  int width = map_.info.width;
  int height = map_.info.height;
  float resolution = map_.info.resolution;
  float origin_x = map_.info.origin.position.x;
  float origin_y = map_.info.origin.position.y;

  // Převod start a goal do souřadnic mapy
  int startX = (int)((start.pose.position.x - origin_x) / resolution);
  int startY = (int)((start.pose.position.y - origin_y) / resolution);
  int goalX = (int)((goal.pose.position.x - origin_x) / resolution);
  int goalY = (int)((goal.pose.position.y - origin_y) / resolution);

  // Inicializace start buňky
  auto cStart = std::make_shared<Cell>(startX, startY);
  cStart->g = 0.0f;
  cStart->h = std::sqrt(std::pow(goalX - startX, 2) + std::pow(goalY - startY, 2));
  cStart->f = cStart->g + cStart->h;

  std::vector<std::shared_ptr<Cell>> openList;
  std::vector<bool> closedList(map_.info.height * map_.info.width, false);

  openList.push_back(cStart);

  while (!openList.empty() && rclcpp::ok()) {
    // Vyber buňku s nejmenším f
    auto minIt = std::min_element(
      openList.begin(), openList.end(),
      [](const std::shared_ptr<Cell> & a, const std::shared_ptr<Cell> & b) { return a->f < b->f; });
    auto current = *minIt;
    openList.erase(minIt);

    // Přesun do closed listu
    closedList[current->y * width + current->x] = true;

    // Jsme v cíli?
    if (current->x == goalX && current->y == goalY) {
      path_.poses.clear();
      auto node = current;
      while (node != nullptr) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = origin_x + (node->x + 0.5f) * resolution;
        pose.pose.position.y = origin_y + (node->y + 0.5f) * resolution;
        pose.pose.orientation.w = 1.0;
        path_.poses.push_back(pose);
        node = node->parent;
      }
      std::reverse(path_.poses.begin(), path_.poses.end());
      RCLCPP_INFO(get_logger(), "Path found: %zu waypoints.", path_.poses.size());
      return;
    }

    // Expanduj 8 sousedů
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;

        int nx = current->x + dx;
        int ny = current->y + dy;

        // Kontrola hranic mapy
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

        // Pouze volné buňky
        if (map_.data[ny * width + nx] != 0) continue;

        // Přeskočit pokud již v closed listu
        if (closedList[ny * width + nx]) continue;

        float stepCost = (dx != 0 && dy != 0) ? std::sqrt(2.0f) : 1.0f;
        float newG = current->g + stepCost;
        float newH = std::sqrt(std::pow(goalX - nx, 2) + std::pow(goalY - ny, 2));
        float newF = newG + newH;

        // Je soused již v openListu?
        auto it = std::find_if(
          openList.begin(), openList.end(),
          [nx, ny](const std::shared_ptr<Cell> & c) { return c->x == nx && c->y == ny; });

        if (it == openList.end()) {
          auto neighbor = std::make_shared<Cell>(nx, ny);
          neighbor->g = newG;
          neighbor->h = newH;
          neighbor->f = newF;
          neighbor->parent = current;
          openList.push_back(neighbor);
        } else if (newG < (*it)->g) {
          (*it)->g = newG;
          (*it)->h = newH;
          (*it)->f = newF;
          (*it)->parent = current;
        }
      }
    }
  }

  RCLCPP_ERROR(get_logger(), "Unable to plan path.");
}

void PlanningNode::smoothPath()
{
  // add code here

  // ********
  // * Help *
  // ********
  /*
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    ... processing ...
    path_.poses = newPath;
    */
}

Cell::Cell(int c, int r)
{
  x = c;
  y = r;
  f = 0.0f;
  g = 0.0f;
  h = 0.0f;
  parent = nullptr;
}
