#pragma once
#include "Misc/PathfindingDetails.hpp"

class AStarPather
{
public:
    /* 
        The class should be default constructible, so you may need to define a constructor.
        If needed, you can modify the framework where the class is constructed in the
        initialize functions of ProjectTwo and ProjectThree.
    */

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
    /* ************************************************** */

    /*
        You should create whatever functions, variables, or classes you need.
        It doesn't all need to be in this header and cpp, structure it whatever way
        makes sense to you.
    */

private:
    struct Node {
        Node* parent = nullptr;         // Parent node
        GridPos gridPos;	            // Grid position
        float finalCost = 0.0f;	        // F = G + H
        float givenCost = 0.0f;	        // G
        bool onClosedList = false;
        bool onOpenList = false;
        bool operator<(const Node& other) const { return finalCost > other.finalCost; }
    };

    std::vector<Node> nodes;
    std::vector<std::vector<Node*>> bucketOpenList; // Open list
    std::vector<std::vector<GridPos>> precomputedNeighbors;

    const int NUM_BUCKET = 80;
    const float BUCKET_RANGE = 1.5f;

    // Helper functions
    void onMapChange();
    int getBucketIndex(float fCost) const;
    float calculateHeuristic(const GridPos& start, const GridPos& goal, PathRequest& request) const;
    PathResult reconstructPath(Node* goalNode, PathRequest& request) const;


};