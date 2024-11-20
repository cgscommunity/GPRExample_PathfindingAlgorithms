#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <limits>
#include <stack>
#include <queue>
#include <cmath>
#include <windows.h> // Only for Windows

const int MAX_ROWS = 100; // Maximum dimensions for the maze
const int MAX_COLS = 100;
const int INF = (std::numeric_limits<int>::max)(); // Prevent macro conflicts
const int DIRECTIONS[4][2] = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} }; // Right, Down, Left, Up

// Function declarations
void clearScreen();
void loadMaze(const std::string& filename, char maze[MAX_ROWS][MAX_COLS], int& rows, int& cols, int& startX, int& startY, int& endX, int& endY);
void drawMaze(const char maze[MAX_ROWS][MAX_COLS], int rows, int cols);
void dijkstraWithVisualization(char maze[MAX_ROWS][MAX_COLS], int rows, int cols, int startX, int startY, int endX, int endY, std::chrono::milliseconds stepRate);
void dfsWithVisualization(char maze[MAX_ROWS][MAX_COLS], int rows, int cols, int startX, int startY, int endX, int endY, std::chrono::milliseconds stepRate);
void bfsWithVisualization(char maze[MAX_ROWS][MAX_COLS], int rows, int cols, int startX, int startY, int endX, int endY, std::chrono::milliseconds stepRate);
void aStarWithVisualization(char maze[MAX_ROWS][MAX_COLS], int rows, int cols, int startX, int startY, int endX, int endY, std::chrono::milliseconds stepRate);

// Main function
int main()
{
    char maze[MAX_ROWS][MAX_COLS];
    int rows, cols, startX, startY, endX, endY;

    // Load the maze from a file
    loadMaze("maze.txt", maze, rows, cols, startX, startY, endX, endY);

    // Prompt the user to enter the step rate
    int stepRateInput;
    std::cout << "Enter the step rate in milliseconds (1 to 1000): ";
    std::cin >> stepRateInput;

    // Validate the step rate input
    if (stepRateInput < 1 || stepRateInput > 1000)
    {
        std::cerr << "Invalid step rate. Please enter a value between 1 and 1000." << std::endl;
        return 1;
    }
    std::chrono::milliseconds stepRate(stepRateInput);

    // Ask the user to select an algorithm
    std::cout << "Select an algorithm to solve the maze:\n";
    std::cout << "1. Dijkstra\n";
    std::cout << "2. Depth-First Search (DFS)\n";
    std::cout << "3. Breadth-First Search (BFS)\n";
    std::cout << "4. A* Search\n";
    int choice;
    std::cin >> choice;

    // Run the selected algorithm
    switch (choice)
    {
        case 1:
            dijkstraWithVisualization(maze, rows, cols, startX, startY, endX, endY, stepRate);
            break;
        case 2:
            dfsWithVisualization(maze, rows, cols, startX, startY, endX, endY, stepRate);
            break;
        case 3:
            bfsWithVisualization(maze, rows, cols, startX, startY, endX, endY, stepRate);
            break;
        case 4:
            aStarWithVisualization(maze, rows, cols, startX, startY, endX, endY, stepRate);
            break;
        default:
            std::cerr << "Invalid choice. Exiting." << std::endl;
    }

    return 0;
}

// Function to clear the console screen
void clearScreen()
{
    system("cls");
}

// Function to load the maze into a 2D array
void loadMaze(const std::string& filename, char maze[MAX_ROWS][MAX_COLS], int& rows, int& cols, int& startX, int& startY, int& endX, int& endY)
{
    std::ifstream file(filename);
    if (!file)
    {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        exit(1);
    }

    rows = 0;
    std::string line;
    while (std::getline(file, line))
    {
        cols = line.length();
        for (int i = 0; i < cols; ++i)
        {
            maze[rows][i] = line[i];
            if (line[i] == 'S')
            {
                startX = i;
                startY = rows;
            }
            if (line[i] == 'E')
            {
                endX = i;
                endY = rows;
            }
        }
        ++rows;
    }
}

// Function to draw the maze
void drawMaze(const char maze[MAX_ROWS][MAX_COLS], int rows, int cols)
{
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            std::cout << maze[i][j];
        }
        std::cout << std::endl;
    }
}

// Dijkstra's algorithm with visualization
void dijkstraWithVisualization(char maze[MAX_ROWS][MAX_COLS], int rows, int cols, int startX, int startY, int endX, int endY, std::chrono::milliseconds stepRate)
{
    int dist[MAX_ROWS][MAX_COLS];
    bool visited[MAX_ROWS][MAX_COLS] = { false };
    int steps = 0;

    // Initialize distances
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            dist[i][j] = INF;

    dist[startY][startX] = 0;

    while (true)
    {
        int minDist = INF, currentX = -1, currentY = -1;

        // Find the unvisited node with the smallest distance
        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                if (!visited[i][j] && dist[i][j] < minDist)
                {
                    minDist = dist[i][j];
                    currentX = j;
                    currentY = i;
                }
            }
        }

        // If no valid node is found, stop
        if (currentX == -1 || currentY == -1)
        {
            std::cout << "No more nodes to visit. Pathfinding complete!" << std::endl;
            break;
        }

        // Mark the current node as visited
        visited[currentY][currentX] = true;

        // Mark the current node on the maze
        if (maze[currentY][currentX] != 'S' && maze[currentY][currentX] != 'E')
        {
            maze[currentY][currentX] = '.';
        }

        // Clear the screen and redraw the maze
        clearScreen();
        drawMaze(maze, rows, cols);
        std::this_thread::sleep_for(stepRate);

        // Increment step counter
        steps++;

        // Stop if we reached the end
        if (currentX == endX && currentY == endY)
        {
            std::cout << "Reached the end point using Dijkstra in " << steps << " steps!" << std::endl;
            return;
        }

        // Update distances for neighbors
        for (const auto& dir : DIRECTIONS)
        {
            int nx = currentX + dir[0];
            int ny = currentY + dir[1];
            if (nx >= 0 && nx < cols && ny >= 0 && ny < rows &&
                maze[ny][nx] != '#' && !visited[ny][nx])
            {
                int newDist = dist[currentY][currentX] + 1;
                if (newDist < dist[ny][nx])
                {
                    dist[ny][nx] = newDist;
                }
            }
        }
    }
}

// Depth-First Search (DFS) with visualization
void dfsWithVisualization(char maze[MAX_ROWS][MAX_COLS], int rows, int cols, int startX, int startY, int endX, int endY, std::chrono::milliseconds stepRate)
{
    std::stack<std::pair<int, int>> stack;
    bool visited[MAX_ROWS][MAX_COLS] = { false };
    stack.push({ startX, startY });
    int steps = 0;

    while (!stack.empty())
    {
        int x = stack.top().first;
        int y = stack.top().second;
        stack.pop();

        if (visited[y][x]) continue;
        visited[y][x] = true;

        if (maze[y][x] != 'S' && maze[y][x] != 'E') maze[y][x] = '.';

        clearScreen();
        drawMaze(maze, rows, cols);
        std::this_thread::sleep_for(stepRate);
        steps++;

        if (x == endX && y == endY)
        {
            std::cout << "Reached the end point using DFS in " << steps << " steps!" << std::endl;
            return;
        }

        for (const auto& dir : DIRECTIONS)
        {
            int nx = x + dir[0];
            int ny = y + dir[1];
            if (nx >= 0 && nx < cols && ny >= 0 && ny < rows &&
                maze[ny][nx] != '#' && !visited[ny][nx])
            {
                stack.push({ nx, ny });
            }
        }
    }

    std::cout << "No path found using DFS!" << std::endl;
}

// Breadth-First Search (BFS) with visualization
void bfsWithVisualization(char maze[MAX_ROWS][MAX_COLS], int rows, int cols, int startX, int startY, int endX, int endY, std::chrono::milliseconds stepRate)
{
    std::queue<std::pair<int, int>> queue;
    bool visited[MAX_ROWS][MAX_COLS] = { false };
    queue.push({ startX, startY });
    visited[startY][startX] = true;
    int steps = 0;

    while (!queue.empty())
    {
        int x = queue.front().first;
        int y = queue.front().second;
        queue.pop();

        if (maze[y][x] != 'S' && maze[y][x] != 'E') maze[y][x] = '.';

        clearScreen();
        drawMaze(maze, rows, cols);
        std::this_thread::sleep_for(stepRate);
        steps++;

        if (x == endX && y == endY)
        {
            std::cout << "Reached the end point using BFS in " << steps << " steps!" << std::endl;
            return;
        }

        for (const auto& dir : DIRECTIONS)
        {
            int nx = x + dir[0];
            int ny = y + dir[1];
            if (nx >= 0 && nx < cols && ny >= 0 && ny < rows &&
                maze[ny][nx] != '#' && !visited[ny][nx])
            {
                queue.push({ nx, ny });
                visited[ny][nx] = true;
            }
        }
    }

    std::cout << "No path found using BFS!" << std::endl;
}

// A* Search (A-Star) with visualization
void aStarWithVisualization(char maze[MAX_ROWS][MAX_COLS], int rows, int cols, int startX, int startY, int endX, int endY, std::chrono::milliseconds stepRate)
{
    struct Node
    {
        int x, y, cost, priority;
        bool operator>(const Node& other) const
        {
            return priority > other.priority;
        }
    };

    auto manhattanDistance = [](int x1, int y1, int x2, int y2)
    {
        return std::abs(x1 - x2) + std::abs(y1 - y2);
    };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    bool visited[MAX_ROWS][MAX_COLS] = { false };
    pq.push({ startX, startY, 0, manhattanDistance(startX, startY, endX, endY) });
    int steps = 0;

    while (!pq.empty())
    {
        Node current = pq.top();
        pq.pop();
        int x = current.x, y = current.y;

        if (visited[y][x]) continue;
        visited[y][x] = true;

        if (maze[y][x] != 'S' && maze[y][x] != 'E') maze[y][x] = '.';

        clearScreen();
        drawMaze(maze, rows, cols);
        std::this_thread::sleep_for(stepRate);
        steps++;

        if (x == endX && y == endY)
        {
            std::cout << "Reached the end point using A* in " << steps << " steps!" << std::endl;
            return;
        }

        for (const auto& dir : DIRECTIONS)
        {
            int nx = x + dir[0];
            int ny = y + dir[1];
            if (nx >= 0 && nx < cols && ny >= 0 && ny < rows &&
                maze[ny][nx] != '#' && !visited[ny][nx])
            {
                int newCost = current.cost + 1;
                int priority = newCost + manhattanDistance(nx, ny, endX, endY);
                pq.push({ nx, ny, newCost, priority });
            }
        }
    }

    std::cout << "No path found using A*!" << std::endl;
}
