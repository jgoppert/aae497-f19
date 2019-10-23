#include <iostream>
#include <vector>

using namespace std;

class movie
{

public:
    std::string m_title;
    int m_year;
    float m_rating;
    std::vector<std::string> m_actor;
    std::string m_genre{"horror"};

    void print() {
        std::cout << m_title << "generated in" << m_year << std::endl;
    }
    movie(std::string title, int year, float rating, std::vector<std::string> actor, std::string genre):
        m_title(title), m_year(year), m_rating(rating), m_genre(genre)
    {
    }
    virtual ~movie() {
    }
};

int main(int argc, char const *argv[])
{
    movie test1("movie",1975,9.5,{"A","B"},"Comedy");
    movie test2("drama",2019,5.5, {"C","D"},"history");
    std::cout << test1.m_title << " is made in " << test1.m_year << " with rating " << test1.m_rating << std::endl;
    std::cout << test2.m_title << " is made in " << test2.m_year << " with rating " << test2.m_rating << std::endl;

    return 0;
}
