#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

const string M50_1[] = {
        "/Users/makito/Temp/Astar/Map50_1.bmp",
        "/Users/makito/Temp/Astar/Map50_1_Out.png",
        "/Users/makito/Temp/Astar/Map50_1_Path.png"
};

const string M50_2[] = {
        "/Users/makito/Temp/Astar/Map50_2.bmp",
        "/Users/makito/Temp/Astar/Map50_2_Out.png",
        "/Users/makito/Temp/Astar/Map50_2_Path.png"
};

const string M50_3[] = {
        "/Users/makito/Temp/Astar/Map50_3.bmp",
        "/Users/makito/Temp/Astar/Map50_3_Out.png",
        "/Users/makito/Temp/Astar/Map50_3_Path.png"
};

const string BRAIN_FUCKING[] = {
        "/Users/makito/Temp/Astar/Map_Brain_Fucking.png",
        "/Users/makito/Temp/Astar/Map_Brain_Fucking_Out.png",
        "/Users/makito/Temp/Astar/Map_Brain_Fucking_Path.png"
};

const int ALLOW_VERTEX_PASSTHROUGH = 1;
const int NODE_FLAG_CLOSED = -1;
const int NODE_FLAG_UNDEFINED = 0;
const int NODE_FLAG_OPEN = 1;

const int NODE_TYPE_ZERO = 0;
const int NODE_TYPE_OBSTACLE = 1;
const int NODE_TYPE_START = 2;
const int NODE_TYPE_END = 3;

const int G_DIRECT = 10;
const int G_SKEW = 14;

const string *FILE_PATH = M50_3;

class MapNode {
public:
    int x = -1;
    int y = -1;
    int h = 0;
    int g = 0;
    int type = NODE_TYPE_ZERO;
    int flag = NODE_FLAG_UNDEFINED;
    MapNode *parent = 0;

    MapNode() { }

    MapNode(int x, int y, int type = NODE_TYPE_ZERO, int flag = NODE_FLAG_UNDEFINED, MapNode *parent = 0) {
        this->x = x;
        this->y = y;
        this->type = type;
        this->flag = flag;
        this->parent = parent;
    }

    int f() {
        return g + h;
    }
};

class MapSize {
public:
    unsigned long width = 0;
    unsigned long height = 0;
    unsigned long size = 0;

    MapSize() { }

    MapSize(unsigned long width, unsigned long height) {
        this->width = width;
        this->height = height;
        this->size = width * height;
    }
};

Mat map;
MapSize mapSize;
vector<MapNode> mapData;
vector<MapNode *> openList;

MapNode *startNode;
MapNode *targetNode;

MapNode *mapAt(int x, int y);

vector<MapNode *> neighbors(MapNode *node);

int computeH(MapNode *node1, MapNode *node2);

int computeG(MapNode *node1, MapNode *node2);

vector<MapNode *> find();

void drawPath(Mat &map, vector<MapNode *> path);

void drawOpenList();

/**	The standard heuristic is the Manhattan distance.
 *	Look at your cost function and see what the least cost is
 *	for moving from one space to another.
 *	The heuristic should be cost times manhattan distance: */
inline int manhattan_distance(MapNode* node1, MapNode* node2){
    return abs(node2->x - node1->x) + abs(node2->y - node1->y);
}

/**	If on your map you allow diagonal movement, then you need a different heuristic.
 *	The Manhattan distance for (4 east, 4 north) will be 8.
 *	However, you could simply move (4 northeast) instead, so the heuristic should be 4.
 *	This function handles diagonals: */
inline int diagonal_distance(MapNode* node1, MapNode* node2){
    return max(abs(node2->x - node1->x),abs(node2->y - node1->y));
}

int main() {
    map = imread(FILE_PATH[0]);
    Mat resized;
    resize(map, resized, Size(500, 500), 0, 0, INTER_NEAREST);
    imwrite(FILE_PATH[1], resized);

    mapSize = MapSize(map.cols, map.rows);
    mapData = vector<MapNode>(mapSize.size);
    cout << "MapSize(" << mapSize.width << ", " << mapSize.height << ", " << mapSize.size << ")" << endl;
    for (int y = 0; y < map.rows; y++) {
        for (int x = 0; x < map.cols; x++) {
            if (map.at<Vec3b>(y, x) == Vec3b(255, 255, 255)) {
                mapData[y * mapSize.width + x] = MapNode(x, y, NODE_TYPE_ZERO);
            } else if (map.at<Vec3b>(y, x) == Vec3b(0, 0, 0)) {
                mapData[y * mapSize.width + x] = MapNode(x, y, NODE_TYPE_OBSTACLE);
            } else if (map.at<Vec3b>(y, x) == Vec3b(255, 0, 0)) {
                MapNode node(x, y, NODE_TYPE_START);
                mapData[y * mapSize.width + x] = node;
                startNode = &mapData[y * mapSize.width + x];
            } else if (map.at<Vec3b>(y, x) == Vec3b(0, 0, 255)) {
                MapNode node(x, y, NODE_TYPE_END);
                mapData[y * mapSize.width + x] = node;
                targetNode = &mapData[y * mapSize.width + x];
            } else {
                map.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
                mapData[y * mapSize.width + x] = MapNode(x, y, NODE_TYPE_OBSTACLE);
            }
        }
    }

    for (int y = 0; y < mapSize.height; y++) {
        for (int x = 0; x < mapSize.width; x++) {
            cout << mapAt(x, y)->type << " ";
        }
        cout << endl;
    }

    int i;
    cout << "startNode=(" << startNode->x << ", " << startNode->y << ")" << endl;
    cout << "endNode=(" << targetNode->x << ", " << targetNode->y << ")" << endl;
    cout << ((i = 1) == 1) << endl;

    openList.push_back(startNode);
    vector<MapNode *> path = find();

    drawPath(map, path);
    imwrite(FILE_PATH[2], map);

    return 0;
}

void drawPath(Mat &map, vector<MapNode *> path) {
    cvtColor(map, map, COLOR_BGR2HSV);
    for (int i = 0; i < path.size() - 1; i++) {
        MapNode *node = path[i];
        map.at<Vec3b>(node->y, node->x) = Vec3b(20 + (1.0 - ((double) i / path.size())) * 80, 200, 255);
        cout << "->(" << node->x << "," << node->y << ")";
    }
    cout << endl;

    cvtColor(map, map, COLOR_HSV2BGR);
    resize(map, map, Size(500, 500), 0, 0, INTER_NEAREST);
}

void drawOpenList() {
    for (int i = 0; i < openList.size(); i++) {
        MapNode *node = openList[i];
        if (node == startNode || node == targetNode)continue;
        map.at<Vec3b>(node->y, node->x) = Vec3b(210, 210, 210);
    }
}

vector<MapNode *> find() {
    vector<MapNode *> path;
    cout << "Finding started!" << endl;
    int iteration = 0;
    MapNode *node;
    MapNode *reversedPtr = 0;
    while (openList.size() > 0) {
        node = openList.at(0);

        for (int i = 0, max = openList.size(); i < max; i++) {
            if (openList[i]->f() <= node->f() && openList[i]->h < node->h) {
                node = openList[i];
            }
        }
        openList.erase(remove(openList.begin(), openList.end(), node), openList.end());
        node->flag = NODE_FLAG_CLOSED;
        cout << iteration++ << endl;
        cout << "   Current node " << node->x << ", " << node->y << " ..." << endl;
        if (node->parent != 0)
            cout << "       ... parent " << node->parent->x << ", " << node->parent->y << endl;
        if (node == targetNode) {
            cout << "Reached the target node." << endl;
            reversedPtr = node;
            break;
        }
        vector<MapNode *> neighborNodes = neighbors(node);
        cout << "       ... has " << neighborNodes.size() << " neighbors" << endl;
        for (int i = 0; i < neighborNodes.size(); i++) {
            MapNode *_node = neighborNodes[i];
            if (_node->flag == NODE_FLAG_CLOSED || _node->type == NODE_TYPE_OBSTACLE) {
                continue;
            }
            int g = node->g + computeG(_node, node);
            if (_node->flag == NODE_FLAG_UNDEFINED || g < _node->g) {
                _node->g = g;
                _node->h = computeH(_node, targetNode);
                _node->parent = node;
                if (_node->flag != NODE_FLAG_OPEN) {
                    _node->flag = NODE_FLAG_OPEN;
                    openList.push_back(_node);
                }
            }
        }
        drawOpenList();
        if (openList.size() <= 0) break;

    }
    if (reversedPtr == 0) {
        cout << "Target node is unreachable." << endl;
    } else {
        MapNode *_node = reversedPtr;
        while (_node->parent != 0) {
            path.push_back(_node);
            _node = _node->parent;
        }
        reverse(path.begin(), path.end());
    }
    return path;
}

vector<MapNode *> neighbors(MapNode *node) {
    vector<MapNode *> available;
    MapNode *_node;

    // L
    if ((_node = mapAt(node->x - 1, node->y)) != 0)available.push_back(_node);
    // T
    if ((_node = mapAt(node->x, node->y - 1)) != 0)available.push_back(_node);
    // R
    if ((_node = mapAt(node->x + 1, node->y)) != 0)available.push_back(_node);
    // B
    if ((_node = mapAt(node->x, node->y + 1)) != 0)available.push_back(_node);

    if (ALLOW_VERTEX_PASSTHROUGH) {
        // LT
        if ((_node = mapAt(node->x - 1, node->y - 1)) != 0)available.push_back(_node);
        // RT
        if ((_node = mapAt(node->x + 1, node->y - 1)) != 0)available.push_back(_node);
        // RB
        if ((_node = mapAt(node->x + 1, node->y + 1)) != 0)available.push_back(_node);
        // LB
        if ((_node = mapAt(node->x - 1, node->y + 1)) != 0)available.push_back(_node);
    }

    return available;
}

int computeH(MapNode *node1, MapNode *node2) {
    // return abs(node1->x - node2->x) + abs(node1->y - node2->y);
    if (ALLOW_VERTEX_PASSTHROUGH) {
        return diagonal_distance(node1, node2)*G_SKEW;
    } else {
        return manhattan_distance(node1, node2)*G_DIRECT;
    }
}

int computeG(MapNode *node1, MapNode *node2) {
    int dX = abs(node1->x - node2->x);
    int dY = abs(node1->y - node2->y);
    if (dX > dY) {
        return 14 * dY + 10 * (dX - dY);
    } else {
        return 14 * dX + 10 * (dY - dX);
    }
}

MapNode *mapAt(int x, int y) {
    if (x < 0 || y < 0 || x >= mapSize.width || y >= mapSize.height)return 0;
    return &mapData[y * mapSize.width + x];
}