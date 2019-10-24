#include <iostream>
#include <vector>

class Movie {
public:
    std::string m_title{""};
    int m_year{1900};
    float m_rating{0};
    std::vector<std::string> m_actors {};
    std::string m_genre{"horror"};
    void print() {
        std::cout << m_title << m_year << std::endl;
    }
    Movie (std::string title, int year, float rating, std::vector<std::string> actors, std::string genre):
        // member initialization list (a way to avoid constructing twice)
        m_title(title), m_year(year), m_rating(rating), m_actors(actors), m_genre(genre)
    {
    }
    virtual ~Movie () {
    }
};

int main(int argc, char const *argv[])
{
    Movie m1("Monty Python and the Holy Grail", 1975, 8.5, {"Graham Chapman", "John Cleese"}, "Comedy");
    Movie m2("Aliens", 1986, 8.4, {"Sigourney Weaver", "Newt"}, "Horror");

    std::list<Movie> database;
    database.insert(m1);
    database.insert(m2);
    
    std::cout << "hello world " << m1.m_rating << std::endl;
    /* code */
    return 0;
}
