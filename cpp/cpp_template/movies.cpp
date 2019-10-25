#include <iostream>
#include <vector>
<<<<<<< HEAD
=======
#include <list>
#include <memory>
>>>>>>> eb4087acefb7e3964fbf7138371e79e42431d5e2

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
<<<<<<< HEAD
    Movie (std::string title, int year, float rating, std::vector<std::string> actors, std::string genre):
        // member initialization list (a way to avoid constructing twice)
        m_title(title), m_year(year), m_rating(rating), m_actors(actors), m_genre(genre)
    {
    }
    virtual ~Movie () {
=======
    Movie(std::string title, int year, float rating,
        std::vector<std::string> actors, std::string genre) :
        // member initialization list (a way to avoid constructing twice)
        m_title(title), m_year(year), m_rating(rating), m_actors(actors),
        m_genre(genre)
    {
    }
    virtual ~Movie() {
>>>>>>> eb4087acefb7e3964fbf7138371e79e42431d5e2
    }
};

int main(int argc, char const *argv[])
{
<<<<<<< HEAD
    Movie m1("Monty Python and the Holy Grail", 1975, 8.5, {"Graham Chapman", "John Cleese"}, "Comedy");
    Movie m2("Aliens", 1986, 8.4, {"Sigourney Weaver", "Newt"}, "Horror");
=======
    auto m1 = std::make_shared<Movie>("Monty Python and the Holy Grail",
        1975, 8.5, {"Graham Chapman", "John Cleese"}, "Comedy");
    auto m2 = std::maked_shared<Movie>("Aliens",
        1986, 8.4, {"Sigourney Weaver", "Newt"}, "Horror");
>>>>>>> eb4087acefb7e3964fbf7138371e79e42431d5e2

    std::list<Movie> database;
    database.insert(m1);
    database.insert(m2);
<<<<<<< HEAD
    
    std::cout << "hello world " << m1.m_rating << std::endl;
    /* code */
    return 0;
}
=======


    std::cout << "hello world" << std::endl;
    /* code */
    return 0;
}
>>>>>>> eb4087acefb7e3964fbf7138371e79e42431d5e2
