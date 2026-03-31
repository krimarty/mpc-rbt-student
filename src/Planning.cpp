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
  path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", 10);

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
    dilateMap();
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to receive map.");
  }
}

void PlanningNode::planPath(
  const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
  std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
{
    if (map_.data.empty()) {
      RCLCPP_WARN(get_logger(), "Map not yet received, cannot plan path.");
      return;
    }

    path_.poses.clear();

    RCLCPP_INFO(get_logger(), "planPath called. Map size: %d x %d, data: %zu cells",
      map_.info.width, map_.info.height, map_.data.size());
    RCLCPP_INFO(get_logger(), "Start: (%.2f, %.2f), Goal: (%.2f, %.2f)",
      request->start.pose.position.x, request->start.pose.position.y,
      request->goal.pose.position.x, request->goal.pose.position.y);

    aStar(request->start, request->goal);
    smoothPath();

    path_.header.frame_id = "map";
    path_.header.stamp = now();
    RCLCPP_INFO(get_logger(), "Path has %zu poses.", path_.poses.size());
    response->plan = path_;
    path_pub_->publish(path_);
}

void PlanningNode::dilateMap()
{
  const float robot_width = 0.5f; // m
  const float resolution = map_.info.resolution;
  const int width = map_.info.width;
  const int height = map_.info.height;

  // Radius robota v buňkách mapy
  int radius = (int)std::ceil((robot_width / 2.0f) / resolution) * 2; // rozsireno o 2 pro jistotu

  // Vytvoreni masky pro dilataci
  std::vector<std::pair<int, int>> mask;
  for (int dy = -radius; dy <= radius; dy++) {
    for (int dx = -radius; dx <= radius; dx++) {
      if (dx * dx + dy * dy <= radius * radius) {
        mask.emplace_back(dx, dy);
      }
    }
  }

  dilated_map_ = map_;

  // Otisknuout masku pokud je buňka obsazená
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (map_.data[y * width + x] > 0) {
        for (const auto & [dx, dy] : mask) {
          int nx = x + dx;
          int ny = y + dy;
          if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            dilated_map_.data[ny * width + nx] = 100;
          }
        }
      }
    }
  }
}

void PlanningNode::aStar(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  int width = dilated_map_.info.width;
  int height = dilated_map_.info.height;
  float resolution = dilated_map_.info.resolution;
  float origin_x = dilated_map_.info.origin.position.x;
  float origin_y = dilated_map_.info.origin.position.y;

  RCLCPP_INFO(get_logger(), "Map origin: (%.2f, %.2f), resolution: %.3f", origin_x, origin_y, resolution);

  // Převod start a goal do souřadnic mapy
  int startX = (int)((start.pose.position.x - origin_x) / resolution);
  int startY = (int)((start.pose.position.y - origin_y) / resolution);
  int goalX = (int)((goal.pose.position.x - origin_x) / resolution);
  int goalY = (int)((goal.pose.position.y - origin_y) / resolution);

  RCLCPP_INFO(get_logger(), "Start cell: (%d, %d) value: %d, Goal cell: (%d, %d) value: %d",
    startX, startY, dilated_map_.data[startY * width + startX],
    goalX, goalY, dilated_map_.data[goalY * width + goalX]);

  // Inicializace start buňky
  auto cStart = std::make_shared<Cell>(startX, startY);
  cStart->g = 0.0f;
  cStart->h = std::sqrt(std::pow(goalX - startX, 2) + std::pow(goalY - startY, 2));
  cStart->f = cStart->g + cStart->h;

  std::vector<std::shared_ptr<Cell>> openList;
  std::vector<bool> closedList(dilated_map_.info.height * dilated_map_.info.width, false);

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
        if (dilated_map_.data[ny * width + nx] != 0) continue;

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
  const float alpha = 0.1f;  // přitahování k originálu
  const float beta = 0.1f;   // přitahování k sousedům
  const float tolerance = 1e-6f;
  const int max_iterations = 1000;

  std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
  int n = newPath.size();

  if (n < 3) return;  // nic k vyhlazení

  for (int iter = 0; iter < max_iterations; iter++) {
    float change = 0.0f;
    for (int i = 1; i < n - 1; i++) {
      float dx, dy;

      // Přitahuj k originálu
      dx = alpha * (path_.poses[i].pose.position.x - newPath[i].pose.position.x);
      dy = alpha * (path_.poses[i].pose.position.y - newPath[i].pose.position.y);
      newPath[i].pose.position.x += dx;
      newPath[i].pose.position.y += dy;
      change += std::abs(dx) + std::abs(dy);

      // Přitahuj k sousedům
      dx = beta * (newPath[i-1].pose.position.x + newPath[i+1].pose.position.x - 2.0f * newPath[i].pose.position.x);
      dy = beta * (newPath[i-1].pose.position.y + newPath[i+1].pose.position.y - 2.0f * newPath[i].pose.position.y);
      newPath[i].pose.position.x += dx;
      newPath[i].pose.position.y += dy;
      change += std::abs(dx) + std::abs(dy);
    }
    if (change < tolerance) break;
  }

  path_.poses = newPath;
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
