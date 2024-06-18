#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}

bool ProjectTwo::implemented_jps_plus()
{
    return false;
}
#pragma endregion

bool AStarPather::initialize()
{
    // handle any one-time setup requirements you have

    /*
        If you want to do any map-preprocessing, you'll need to listen
        for the map change message.  It'll look something like this:

        Callback cb = std::bind(&AStarPather::your_function_name, this);
        Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

        There are other alternatives to using std::bind, so feel free to mix it up.
        Callback is just a typedef for std::function<void(void)>, so any std::invoke'able
        object that std::function can wrap will suffice.
    */
    Callback cb = std::bind(&AStarPather::onMapChange, this);
    Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

    //Callback cb2 = std::bind(&AStarPather::onPathRequestBegin, this, std::placeholders::_1);
    //Messenger::listen_for_message(Messages::PATH_REQUEST_BEGIN, cb2);

    //nodes.resize(terrain->maxMapHeight * terrain->maxMapWidth);
    //bucketOpenList.resize(NUM_BUCKETS);
    //onMapChange();

    return true; // return false if any errors actually occur, to stop engine initialization
}

int AStarPather::getBucketIndex(float fCost) const
{
	return static_cast<int>(fCost / BUCKET_RANGE);
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */
}

void AStarPather::onMapChange() {
    if (!terrain) return;
    int mapWidth = terrain->get_map_width();
    int mapHeight = terrain->get_map_height();
    precomputedNeighbors.resize(mapWidth * mapHeight);

    for (int row = 0; row < mapHeight; ++row) {
        for (int col = 0; col < mapWidth; ++col) {
            GridPos pos { row, col };
            if (!terrain->is_wall(pos)) {
                std::vector<GridPos> neighbors;

                // Check all 8 directions
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        if (dx == 0 && dy == 0) {
                            continue;
                        }

                        GridPos neighborPos{ row + dy, col + dx };
                        if (terrain->is_valid_grid_position(neighborPos) && !terrain->is_wall(neighborPos)) {
                            neighbors.push_back(neighborPos);
                        }
                    }
                }
                precomputedNeighbors[row * mapWidth + col] = neighbors;
            }
        }
    }
}

float AStarPather::calculateHeuristic(const GridPos& pos, const GridPos& goal, PathRequest& request) const {
    switch (request.settings.heuristic) {
        case Heuristic::EUCLIDEAN:
			return sqrtf(static_cast<float>((pos.row - goal.row) * (pos.row - goal.row) + (pos.col - goal.col) * (pos.col - goal.col)));
        case Heuristic::MANHATTAN:
            return static_cast<float>(abs(pos.row - goal.row) + abs(pos.col - goal.col));
        case Heuristic::OCTILE:
			return static_cast<float>(std::max(abs(pos.row - goal.row), abs(pos.col - goal.col)) + (sqrtf(2.0f) - 1.0f) * std::min(abs(pos.row - goal.row), abs(pos.col - goal.col)));
        case Heuristic::CHEBYSHEV:
			return static_cast<float>(std::max(abs(pos.row - goal.row), abs(pos.col - goal.col)));
		case Heuristic::INCONSISTENT:
			return static_cast<float>(rand() % 100);
		default:
			return 0.0f;

    }
}

PathResult AStarPather::reconstructPath(Node* goalNode, PathRequest& request) const {
    request.path.clear();

    Node* currentNode = goalNode;
    while (currentNode->parent != nullptr) {
		request.path.push_back(terrain->get_world_position(currentNode->gridPos));
		currentNode = currentNode->parent;
	}
    std::reverse(request.path.begin(), request.path.end());

	return PathResult::COMPLETE;
}

PathResult AStarPather::compute_path(PathRequest &request)
{
    /*
        This is where you handle pathing requests, each request has several fields:

        start/goal - start and goal world positions
        path - where you will build the path upon completion, path should be
            start to goal, not goal to start
        heuristic - which heuristic calculation to use
        weight - the heuristic weight to be applied
        newRequest - whether this is the first request for this path, should generally
            be true, unless single step is on

        smoothing - whether to apply smoothing to the path
        rubberBanding - whether to apply rubber banding
        singleStep - whether to perform only a single A* step
        debugColoring - whether to color the grid based on the A* state:
            closed list nodes - yellow
            open list nodes - blue

            use terrain->set_color(row, col, Colors::YourColor);
            also it can be helpful to temporarily use other colors for specific states
            when you are testing your algorithms

        method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
            will be A* generally, unless you implement extra credit features

        The return values are:
            PROCESSING - a path hasn't been found yet, should only be returned in
                single step mode until a path is found
            COMPLETE - a path to the goal was found and has been built in request.path
            IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
    */

    // WRITE YOUR CODE HERE
    int mapWidth = terrain->get_map_width();
    int mapHeight = terrain->get_map_height();

    if (request.newRequest) {
        nodes.clear();
        nodes.resize(mapWidth * mapHeight);
        for (auto& bucket : bucketOpenList) {
            bucket.clear();
        }
        bucketOpenList.resize(NUM_BUCKET);

        GridPos start = terrain->get_grid_position(request.start);
        nodes[start.row * mapWidth + start.col].gridPos = start;
        nodes[start.row * mapWidth + start.col].givenCost = 0.0f;
        nodes[start.row * mapWidth + start.col].finalCost = calculateHeuristic(start, terrain->get_grid_position(request.goal), request);
        int bucketIndex = getBucketIndex(nodes[start.row * mapWidth + start.col].finalCost);
        bucketOpenList[bucketIndex].push_back(&nodes[start.row * mapWidth + start.col]);

        if (request.settings.debugColoring) {
            for (int row = 0; row < mapHeight; ++row) {
                for (int col = 0; col < mapWidth; ++col) {
                    terrain->set_color(row, col, terrain->baseColor);
                }
            }
        }
    }

    int currentBucketIndex = 0;
    while (currentBucketIndex < NUM_BUCKET) {
        while (!bucketOpenList[currentBucketIndex].empty()) {
            Node* parentNode = bucketOpenList[currentBucketIndex].back();
            bucketOpenList[currentBucketIndex].pop_back();

            parentNode->onClosedList = true;
            if (request.settings.debugColoring) {
				terrain->set_color(parentNode->gridPos, Colors::Yellow);
			}

            //if (parentNode->parent != nullptr && request.settings.debugColoring) {
            //    terrain->draw(parentPos)

            if (parentNode->gridPos == terrain->get_grid_position(request.goal)) {
                return reconstructPath(parentNode, request);
            }

            // For all neighboring child nodes of parentNode
            for (const GridPos& neighborPos : precomputedNeighbors[parentNode->gridPos.row * mapWidth + parentNode->gridPos.col]) {
                Node* childNode = &nodes[neighborPos.row * mapWidth + neighborPos.col];

                // Compute child node's costs:
                float tentative_gCost = parentNode->givenCost + 1.0f;
                float tentative_fCost = tentative_gCost + calculateHeuristic(neighborPos, terrain->get_grid_position(request.goal), request);

                // If child node is not on the closed list
                if (childNode->onClosedList) {
                    continue;
                }
                else if (childNode->onOpenList) {
                    if (tentative_fCost < childNode->finalCost) {
                        childNode->finalCost = tentative_fCost;
                        childNode->givenCost = tentative_gCost;
                        childNode->parent = parentNode;
                    }
                }
                else {
                    childNode->finalCost = tentative_fCost;
                    childNode->givenCost = tentative_gCost;
                    childNode->parent = parentNode;
                    childNode->onOpenList = true;
                    int bucketIndex = getBucketIndex(childNode->finalCost);
                    bucketOpenList[bucketIndex].push_back(childNode);

                    if (request.settings.debugColoring) {
                        terrain->set_color(childNode->gridPos, Colors::Blue);
                    }
                }
            }
        }
        ++currentBucketIndex;

        

    }
    //// Just sample code, safe to delete
    //GridPos start = terrain->get_grid_position(request.start);
    //GridPos goal = terrain->get_grid_position(request.goal);
    //terrain->set_color(start, Colors::Orange);
    //terrain->set_color(goal, Colors::Orange);
    //request.path.push_back(request.start);
    //request.path.push_back(request.goal);
    return PathResult::IMPOSSIBLE;
}
