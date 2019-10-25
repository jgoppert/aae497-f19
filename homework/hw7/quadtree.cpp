// import relevant libraries
#include <memory>
#include <iostream>
#include <list>
#include <sstream>
#include <exception>
#include <cstdlib> /* srand, rand */
#include <ctime>   /* time */
#include <cmath>   /* sqrt */
#include <chrono>

struct Position
{
    double x, y;
};

struct Landmark
{
    Position pos;
    int id;
};

class QuadTree
{
public:
    QuadTree() = delete;
    virtual ~QuadTree()
    {
        if (m_verbose)
        {
            std::stringstream s;
            s << std::string(m_depth, ' ') << "QuadTree deconstructor"
              << " size: " << m_size
              << " center: " << m_center.x << ", " << m_center.y
              << std::endl;
            std::cout << s.str();
        }
    }
    QuadTree(const Position &center, double size, double resolution, int depth = 0) : m_center(center), m_size(size), m_resolution(resolution), m_depth(depth)
    {
        if (m_verbose)
        {
            std::stringstream s;
            s << std::string(m_depth, ' ') << "QuadTree constructor"
              << " size: " << m_size
              << " resolution: " << m_resolution
              << " center: " << m_center.x << ", " << m_center.y
              << std::endl;
            std::cout << s.str();
        }
    }
    /**
     * This function is is just used to check if a point is actuall contained in the tree
     **/
    bool contains(const Position &p)
    {
        double tol = 1e-5;
        return (
            (p.x > (m_center.x - m_size - tol))
            && (p.x < (m_center.x + m_size + tol))
            && (p.y > (m_center.y - m_size - tol))
            && (p.y < (m_center.y + m_size + tol)));
    }
    /**
     * Allows landmark insertion into QuadTree
     **/
    void insert(const Landmark &lm, int depth = 0)
    {
        if (m_check && !contains(lm.pos))
        {
            std::stringstream s;
            s << "landmark not inside"
              << " x: " << lm.pos.x
              << " y: " << lm.pos.y
              << " size: " << m_size
              << " center: " << m_center.x << ", " << m_center.y
              << std::endl;
            throw std::runtime_error(s.str());
        }
        if (m_size < m_resolution)
        {
            m_landmarks.push_back(lm);
            //std::cout << "resolution reached, adding landmark: " << m_landmarks.size() << std::endl;
        }
        else
        {
            if (lm.pos.x > m_center.x)
            {
                if (lm.pos.y > m_center.y)
                {
                    Position center{m_center.x + m_size / 2, m_center.y + m_size / 2};
                    subInsert(m_NE, lm, center);
                }
                else
                {
                    Position center{m_center.x + m_size / 2, m_center.y - m_size / 2};
                    subInsert(m_SE, lm, center);
                }
            }
            else
            {
                if (lm.pos.y > m_center.y)
                {
                    Position center{m_center.x - m_size / 2, m_center.y + m_size / 2};
                    subInsert(m_NW, lm, center);
                }
                else
                {
                    Position center{m_center.x - m_size / 2, m_center.y - m_size / 2};
                    subInsert(m_SW, lm, center);
                }
            }
        }
    }
    /**
     * search: TODO
     * 
     * This function must recursively search the quadtree to find all landmarks
     * that are within the search radius.
     **/
    std::list<Landmark> search(const Position &position, double radius)
    {
        std::list<Landmark> close_landmarks;
		// just fill in your logic here
        return close_landmarks;
    }

private:
    void subInsert(std::shared_ptr<QuadTree> &tree, const Landmark &lm, const Position &pos)
    {
        if (!tree.get())
        {
            tree = std::make_shared<QuadTree>(pos, m_size / 2, m_depth + 1);
        }
        tree->insert(lm);
    }
    std::shared_ptr<QuadTree> m_NE{}, m_SE{}, m_SW{}, m_NW{};
    std::list<Landmark> m_landmarks{};
    Position m_center{0, 0};
    double m_size{1024};
    double m_resolution{1};
    bool m_check{true};
    int m_depth{0};
    bool m_verbose{false};
};

int main(int argc, char const *argv[])
{
    srand(1234); // seed random number generator

    Position center{0, 0}; // center of space
    double size = 1000;       // size of space
    double resolution = 1; // smallest cell in quadtree
    int n_landmarks = 1000;  // number of landmarks
    QuadTree tree(center, size, resolution);
    double search_radius = 50.0; // radius we want to find landmarks within
    std::cout << "size: " << size << " resolution: " << resolution << " n_landmarks: " << n_landmarks <<  std::endl;

    // create random landmarks
    std::list<Landmark> landmarks;
    for (int id = 0; id < n_landmarks; id++)
    {
        float x = size * 2 * (double(rand()) / RAND_MAX - 0.5);
        float y = size * 2 * (double(rand()) / RAND_MAX - 0.5);
        //std::cout << "inserting landmark id: " << id << " x: " << x << " y: " << y << std::endl;
        landmarks.push_back(Landmark{x, y, id});
    }
    std::cout << "created " << landmarks.size() << " landmarks" << std::endl;

    // where you are
    float x = size * 2 * (double(rand()) / RAND_MAX - 0.5);
    float y = size * 2 * (double(rand()) / RAND_MAX - 0.5);
    Position vehicle_position{x, y};
    std::cout << "searcing at x: " << x << " y: " << y << " radius: " << search_radius << std::endl;

    // brute force search
    std::list<Landmark> close_landmarks_brute_force;
    auto start = std::chrono::high_resolution_clock::now();
    for (auto &lm : landmarks)
    {
        float dx = vehicle_position.x - lm.pos.x;
        float dy = vehicle_position.y - lm.pos.y;
        float d = sqrt(dx * dx + dy * dy);
        if (d < search_radius)
        {
            close_landmarks_brute_force.push_back(lm);
        }
    }
    double elapsed_brute_force_search = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                            std::chrono::high_resolution_clock::now() - start)
                                            .count();
    std::cout << "search landmarks brute force,\telapsed time "
              << elapsed_brute_force_search << " ns" << std::endl;
    // output close landmarks
    for (auto &lm : close_landmarks_brute_force)
    {
        std::cout << "id: " << lm.id << " x: " << lm.pos.x << " y: " << lm.pos.y << std::endl;
    }

    // insert random landmarks into quadtree
    std::cout << "quadtree inserting landmarks";
    start = std::chrono::high_resolution_clock::now();
    for (auto &lm : landmarks)
    {
        tree.insert(lm);
    }
    double elapsed_quadtree_insert = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::high_resolution_clock::now() - start)
                                         .count();
    std::cout << ",\telapsed time " << elapsed_quadtree_insert << " ns" << std::endl;

    // quadtree search
    std::cout << "quadtree searching";
    start = std::chrono::high_resolution_clock::now();
    std::list<Landmark> close_landmarks_quadtree = tree.search(vehicle_position, search_radius);
    double elapsed_quadtree_search = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::high_resolution_clock::now() - start)
                                         .count();
    std::cout << ",\t\telapsed time " << elapsed_quadtree_search << " ns" << std::endl;
    for (auto &lm : close_landmarks_quadtree)
    {
        std::cout << "id: " << lm.id << " x: " << lm.pos.x << " y: " << lm.pos.y << std::endl;
    }

    std::cout << "quadtree speed up: " << elapsed_brute_force_search/elapsed_quadtree_search << std::endl;
    return 0;
}
