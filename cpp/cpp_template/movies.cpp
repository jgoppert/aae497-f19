#include <iostream>
#include <vector>

class Movie {
public:
    std::string title;
    int m_year{1900};
    float m_rating{0};
    std::vector<std::string> _actors;
    void print() {
        std::cout << m_title << m_year << std::endl;
    }      
    Movie(std::string title, int year, float rating, std::vector<std::string> actors, 
        std::string genre) :
    m_title(title), m_year(year), m_rating(rating), m_actors(actors)
}
int main(int argc, char const *argv[])
{
    Movie("Monty Python and the Holy Grail", 1975, 8.5,
        {"Graham Chapman","John Cleese"}, "Comedy");
    std::cout << "hello world" << std::endl;
    /* code */
    return 0;
}
