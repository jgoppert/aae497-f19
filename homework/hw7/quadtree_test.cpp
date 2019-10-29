// import relevant libraries
#include <memory>
#include <iostream>
#include <fstream>
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
//     enum CornerType {CORNER_TL, CORNER_TR, CORNER_BL, CORNER_BR};
// private:
//     QuadTree* m_pChild[4]; // 자식노드 정의
//     float m_nCenter;
//     float m_nCorner[4];

// private:
//     QuadTree* _AddChild(float nCornerTL, int nCornerTR, int nCornerBL, int nCornerBR);


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
     * This function is just used to check if a point is actually contained in the tree
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
        if (m_size < m_resolution)  // break 문
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
        std::list<Landmark> empty_land;

        float dx = position.x - m_center.x;
        float dy = position.y - m_center.y;
        float d = sqrt(dx*dx + dy*dy);
        bool sub_check = d < (radius + (2*m_size)/sqrt(2));

        if (m_size <= m_resolution)
        {
            for (auto i:m_landmarks)
            {
                close_landmarks.push_back(i);
            }
            return close_landmarks;
        }
        

        if (sub_check && m_NW.get())
        {
            close_landmarks.splice(close_landmarks.begin(),m_NW->search(position, radius));
        }
        if (sub_check && m_NE.get())
        {
            close_landmarks.splice(close_landmarks.begin(),m_NE->search(position, radius));
        }
        if (sub_check && m_SE.get())
        {
            close_landmarks.splice(close_landmarks.begin(),m_SE->search(position, radius));
        }
        if (sub_check && m_SW.get())
        {
            close_landmarks.splice(close_landmarks.begin(),m_SW->search(position, radius));
        }

        return close_landmarks;
    }

private:
    void subInsert(std::shared_ptr<QuadTree> &tree, const Landmark &lm, const Position &pos)
    {
        if (!tree.get()) // 쿼드트리내에 랜드마크가 없을경우
        {
            tree = std::make_shared<QuadTree>(pos, m_size / 2, m_depth + 1);  //새로운 쿼드트리 형성
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
    QuadTree tree(center, size, resolution); // Defining quadtree object
    double search_radius = 50.0; // radius we want to find landmarks within
    std::cout << "size: " << size << " resolution: " << resolution << " n_landmarks: " << n_landmarks <<  std::endl;

    // create random landmarks
    std::list<Landmark> landmarks;
    for (int id = 0; id < n_landmarks; id++)
    {
        float x = size * 2 * (double(rand()) / RAND_MAX - 0.5);
        float y = size * 2 * (double(rand()) / RAND_MAX - 0.5);  //creating random -1000 to 1000 for x and y
        //std::cout << "inserting landmark id: " << id << " x: " << x << " y: " << y << std::endl;
        landmarks.push_back(Landmark{x, y, id}); // landmark struct insertion(푸시백) (x,y,id)
    }
    std::cout << "created " << landmarks.size() << " landmarks" << std::endl;

    // where you are
    float x = size * 2 * (double(rand()) / RAND_MAX - 0.5);
    float y = size * 2 * (double(rand()) / RAND_MAX - 0.5); // x,y 위치 랜덤지정
    Position vehicle_position{x, y}; // x,y 로 vehicle position struct 지정
    std::cout << "searcing at x: " << x << " y: " << y << " radius: " << search_radius << std::endl;

    // brute force search
    std::list<Landmark> close_landmarks_brute_force;
    auto start = std::chrono::high_resolution_clock::now();
    for (auto &lm : landmarks)
    {
        float dx = vehicle_position.x - lm.pos.x;
        float dy = vehicle_position.y - lm.pos.y; // landmark 와 vehicle position 의 x,y 거리차이
        float d = sqrt(dx * dx + dy * dy); // 거리 계산
        if (d < search_radius) // 거리가 search 반경보다 작을경우..
        {
            close_landmarks_brute_force.push_back(lm); // landmark position 데이터 삽입
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

 
/* Quadtree landmark insertion and searching */

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


    // quadtree searching method
    std::cout << "quadtree searching";
    start = std::chrono::high_resolution_clock::now();
    std::list<Landmark> close_landmarks_quadtree = tree.search(vehicle_position, search_radius); // 서칭 메소드 콜 (x,y,id 반환)
    double elapsed_quadtree_search = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::high_resolution_clock::now() - start)
                                         .count();
    std::cout << ",\t\telapsed time " << elapsed_quadtree_search << " ns" << std::endl;



    for (auto &lm : close_landmarks_quadtree) //lm 을 메소드 리턴 value 에 어드레스 패싱
        {
        std::cout << "id: " << lm.id << " x: " << lm.pos.x << " y: " << lm.pos.y << std::endl;
    }

    std::cout << "quadtree speed up: " << elapsed_brute_force_search/elapsed_quadtree_search << std::endl;


    // data export
    std::ofstream myfile;
    myfile.open("myname.txt");
    myfile << elapsed_quadtree_search << std::endl;
    myfile.close();

    return 0;
}
