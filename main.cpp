#include <SFML/Graphics.hpp>  
#include <queue>              
#include <vector>            
#include <iostream>           
#include <cmath>              
#include <stack>              
#include <limits>             

// Constants for the grid dimensions and cell size
const int rows = 20;
const int cols = 20;
const int cellSize = 30;

// Enum for different types of grid cells (e.g., wall, start, end, etc.)
enum CellType { Empty = 0, Wall = 1, Start = 2, End = 3, Visited = 4, Path = 5 };

// 2D grid to store the type of each cell
std::vector<std::vector<CellType>> grid(rows, std::vector<CellType>(cols, Empty));
sf::Vector2i start(-1, -1);  
sf::Vector2i end(-1, -1);    

// 2D grid to keep track of visited cells
std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
// 2D grid to store the previous cell in the path (for reconstructing the path)
std::vector<std::vector<sf::Vector2i>> cameFrom(rows, std::vector<sf::Vector2i>(cols, sf::Vector2i(-1, -1)));

// Struct for comparing two vectors based on a second value (used in priority queues)
struct CompareVector2i {
    bool operator()(const std::pair<sf::Vector2i, int>& a, const std::pair<sf::Vector2i, int>& b) const {
        return a.second > b.second; 
    }
};

// Function to check if a cell is valid (in bounds, not a wall, and not visited)
bool isValid(int x, int y) {
    return x >= 0 && x < rows && y >= 0 && y < cols && grid[x][y] != Wall && !visited[x][y];
}

// Function to reset the grid, clearing visited cells and paths
void resetGrid() {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (grid[i][j] == Visited || grid[i][j] == Path) {
                grid[i][j] = Empty;
            }
        }
    }
    visited.assign(rows, std::vector<bool>(cols, false));
    cameFrom.assign(rows, std::vector<sf::Vector2i>(cols, sf::Vector2i(-1, -1)));
}

// Function to draw the grid on the window
void drawGrid(sf::RenderWindow& window, sf::RectangleShape& cell) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            cell.setPosition(j * cellSize, i * cellSize);

            // Set cell color based on its type
            switch (grid[i][j]) {
            case Wall:      cell.setFillColor(sf::Color::Black); break;
            case Start:     cell.setFillColor(sf::Color::Green); break;
            case End:       cell.setFillColor(sf::Color::Red); break;
            case Visited:   cell.setFillColor(sf::Color::Yellow); break;
            case Path:      cell.setFillColor(sf::Color::Blue); break;
            default:        cell.setFillColor(sf::Color::White); break;
            }
            window.draw(cell);  
        }
    }
}

// Function to trace and draw the path from the end to the start
void drawPath(sf::RenderWindow& window, sf::RectangleShape& cell) {
    sf::Vector2i current = end;
    while (current != start && current != sf::Vector2i(-1, -1)) {
        cell.setPosition(current.y * cellSize, current.x * cellSize);
        cell.setFillColor(sf::Color::Blue);
        window.draw(cell);  
        current = cameFrom[current.x][current.y]; 
    }
}

// Breadth-First Search (BFS) for pathfinding
void bfs(sf::RenderWindow& window, sf::RectangleShape& cell) {
    std::queue<sf::Vector2i> q;  
    resetGrid();                
    q.push(start);
    visited[start.x][start.y] = true;  

    while (!q.empty()) {
        sf::Vector2i current = q.front();
        q.pop();

        if (current == end) {
            std::cout << "Reached the end!" << std::endl;
            return;
        }

        int dx[] = { -1, 1, 0, 0 };
        int dy[] = { 0, 0, -1, 1 };

        for (int i = 0; i < 4; ++i) {
            int newX = current.x + dx[i];
            int newY = current.y + dy[i];

            if (isValid(newX, newY)) {
                visited[newX][newY] = true;
                cameFrom[newX][newY] = current; 
                q.push(sf::Vector2i(newX, newY));

                grid[newX][newY] = Visited;  
            }
        }

        window.clear();
        drawGrid(window, cell);
        drawPath(window, cell);
        window.display();
        sf::sleep(sf::milliseconds(100));  
    }
}

// A* pathfinding algorithm
void aStar(sf::RenderWindow& window, sf::RectangleShape& cell) {
    std::vector<std::vector<int>> fScore(rows, std::vector<int>(cols, std::numeric_limits<int>::max()));  // fScore grid
    resetGrid();

    // Heuristic function for A* (Manhattan distance)
    auto heuristic = [](sf::Vector2i a, sf::Vector2i b) {
        return abs(a.x - b.x) + abs(a.y - b.y);
        };

    fScore[start.x][start.y] = heuristic(start, end);
    std::priority_queue<std::pair<sf::Vector2i, int>, std::vector<std::pair<sf::Vector2i, int>>, CompareVector2i> pq;
    pq.push({ start, fScore[start.x][start.y] });  // Push start node to priority queue

    while (!pq.empty()) {
        sf::Vector2i current = pq.top().first;
        pq.pop();

        if (current == end) {
            std::cout << "Reached the end!" << std::endl;
            return;
        }

        int dx[] = { -1, 1, 0, 0 };
        int dy[] = { 0, 0, -1, 1 };

        for (int i = 0; i < 4; ++i) {
            int newX = current.x + dx[i];
            int newY = current.y + dy[i];

            if (isValid(newX, newY)) {
                int tentativeScore = fScore[current.x][current.y] + 1;  // Calculate tentative score
                if (tentativeScore < fScore[newX][newY]) {
                    fScore[newX][newY] = tentativeScore;  // Update score
                    cameFrom[newX][newY] = current;  // Track path
                    pq.push({ sf::Vector2i(newX, newY), tentativeScore + heuristic(sf::Vector2i(newX, newY), end) });

                    grid[newX][newY] = Visited;  
                }
            }
        }

        window.clear();
        drawGrid(window, cell);
        drawPath(window, cell);
        window.display();
        sf::sleep(sf::milliseconds(100)); 
    }
}

// Dijkstra's algorithm for pathfinding (similar to A* but without the heuristic)
void dijkstra(sf::RenderWindow& window, sf::RectangleShape& cell) {
    std::vector<std::vector<int>> dist(rows, std::vector<int>(cols, std::numeric_limits<int>::max()));  // Distance grid
    resetGrid();

    dist[start.x][start.y] = 0;  
    std::priority_queue<std::pair<sf::Vector2i, int>, std::vector<std::pair<sf::Vector2i, int>>, CompareVector2i> pq;
    pq.push({ start, 0 });  

    while (!pq.empty()) {
        sf::Vector2i current = pq.top().first;
        pq.pop();

        if (current == end) {
            std::cout << "Reached the end!" << std::endl;
            return;
        }

        int dx[] = { -1, 1, 0, 0 };
        int dy[] = { 0, 0, -1, 1 };

        for (int i = 0; i < 4; ++i) {
            int newX = current.x + dx[i];
            int newY = current.y + dy[i];

            if (isValid(newX, newY)) {
                int tentativeDist = dist[current.x][current.y] + 1;  // Calculate new distance
                if (tentativeDist < dist[newX][newY]) {
                    dist[newX][newY] = tentativeDist;  // Update distance
                    cameFrom[newX][newY] = current;  // Track path
                    pq.push({ sf::Vector2i(newX, newY), tentativeDist });

                    grid[newX][newY] = Visited;  
                }
            }
        }

        window.clear();
        drawGrid(window, cell);
        drawPath(window, cell);
        window.display();
        sf::sleep(sf::milliseconds(100));  
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode(cols * cellSize, rows * cellSize), "Maze Pathfinding"); 
    sf::RectangleShape cell(sf::Vector2f(cellSize - 1, cellSize - 1)); 

    // Main event loop
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close(); 

            if (event.type == sf::Event::MouseButtonPressed) {
                int x = event.mouseButton.x / cellSize;
                int y = event.mouseButton.y / cellSize;

                // Toggle wall placement with left mouse button
                if (event.mouseButton.button == sf::Mouse::Left)
                    grid[y][x] = (grid[y][x] == Wall) ? Empty : Wall;
            }

            // Set start point with 'S', end point with 'E', and run algorithms with keys 'B', 'A', 'D'
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::S) {
                    int x = sf::Mouse::getPosition(window).x / cellSize;
                    int y = sf::Mouse::getPosition(window).y / cellSize;
                    start = sf::Vector2i(y, x);
                    grid[start.x][start.y] = Start;
                }
                if (event.key.code == sf::Keyboard::E) {
                    int x = sf::Mouse::getPosition(window).x / cellSize;
                    int y = sf::Mouse::getPosition(window).y / cellSize;
                    end = sf::Vector2i(y, x);
                    grid[end.x][end.y] = End;
                }
                if (event.key.code == sf::Keyboard::B) {
                    bfs(window, cell);  // Run BFS
                }
                if (event.key.code == sf::Keyboard::A) {
                    aStar(window, cell);  // Run A*
                }
                if (event.key.code == sf::Keyboard::D) {
                    dijkstra(window, cell);  // Run Dijkstra
                }
            }
        }

        window.clear();
        drawGrid(window, cell);
        window.display();
        sf::sleep(sf::milliseconds(100));  
    }

    return 0;
}
