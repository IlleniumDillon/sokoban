#include <list>
#include <algorithm>
#include <iostream>

class GridNode
{
public:
    int x, y;
    float g, h, f;
    GridNode* parent;

    GridNode(int _x, int _y, GridNode* _parent = nullptr) : x(_x), y(_y), parent(_parent), g(0), h(0), f(0) {}
};

class AStar
{
public:
    std::list<GridNode*> openList;
    std::list<GridNode*> closedList;

    bool isInList(std::list<GridNode*> list, GridNode* node)
    {
        for(auto& i : list)
            if(i->x == node->x && i->y == node->y)
                return true;
        return false;
    }

    float heuristic(GridNode* start, GridNode* goal)
    {
        return abs(start->x - goal->x) + abs(start->y - goal->y);
    }

    std::list<GridNode*> findPath(GridNode* start, GridNode* goal)
    {
        openList.push_back(start);
        while(!openList.empty())
        {
            GridNode* current = *std::min_element(openList.begin(), openList.end(), [](GridNode* lhs, GridNode* rhs) { return lhs->f < rhs->f; });
            if(current->x == goal->x && current->y == goal->y)
            {
                std::list<GridNode*> path;
                while(current != nullptr)
                {
                    path.push_front(current);
                    current = current->parent;
                }
                openList.clear();
                closedList.clear();
                return path;
            }
            openList.remove(current);
            closedList.push_back(current);
            for(int dx = -1; dx <= 1; dx++)
            {
                for(int dy = -1; dy <= 1; dy++)
                {
                    GridNode* neighbor = new GridNode(current->x + dx, current->y + dy, current);
                    if(dx == 0 && dy == 0 || isInList(closedList, neighbor))
                        continue;
                    neighbor->g = current->g + 1;
                    neighbor->h = heuristic(neighbor, goal);
                    neighbor->f = neighbor->g + neighbor->h;
                    if(isInList(openList, neighbor))
                    {
                        GridNode* openNode = *std::find_if(openList.begin(), openList.end(), [neighbor](GridNode* node) { return node->x == neighbor->x && node->y == neighbor->y; });
                        if(neighbor->g < openNode->g)
                        {
                            openNode->g = neighbor->g;
                            openNode->f = openNode->g + openNode->h;
                            openNode->parent = current;
                        }
                    }
                    else
                        openList.push_back(neighbor);
                }
            }
        }
        return std::list<GridNode*>();
    }
};