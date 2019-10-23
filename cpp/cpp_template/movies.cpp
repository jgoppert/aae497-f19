#include <iostream>
#include <vector>

class Movie {
public:
    std::string m_title{};
    int m_year{1900};
    float m_rating(0);
    std::vector<std::string> m_actors {};
    std::string genre{"horror"};
    void print() {
        std::cout << m_title << m_year << std::endl;
    }
    Movie(std::string title, int year, float rating,
        std::vector<std::string> actors, std::string genre);
        m_title(title), m_year(year), m_rating(rating)
}

int main(int argc, char const *argv[])
{
    Movie montypython("monty Python and the Hole Grail",
        1975, 8.5, {"Graham"}
}