<<<<<<< HEAD
#include <string>
#include <list>
#include <iostream>

class Movie {
public:
    Movie(std::string title) : m_title(title) {}
    virtual ~Movie() {};
    std::string m_title{""};
=======
#include <iostream>
#include <vector>
#include <list>
#include <memory>

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
    Movie(std::string title, int year, float rating,
        std::vector<std::string> actors, std::string genre) :
        // member initialization list (a way to avoid constructing twice)
        m_title(title), m_year(year), m_rating(rating), m_actors(actors),
        m_genre(genre)
    {
    }
    virtual ~Movie() {
    }
>>>>>>> eb4087acefb7e3964fbf7138371e79e42431d5e2
};

int main(int argc, char const *argv[])
{
<<<<<<< HEAD
    /* code */
    Movie m1("movie 1");
    Movie m2("movie 2");

    std::list<Movie *> database;
    database.emplace_back(&m1);
    database.emplace_back(&m2);

    for (auto movie: database) {
        std::cout << movie->m_title << std::endl;
    }
    return 0;
}
=======
    auto m1 = std::make_shared<Movie>("Monty Python and the Holy Grail",
        1975, 8.5, {"Graham Chapman", "John Cleese"}, "Comedy");
    auto m2 = std::maked_shared<Movie>("Aliens",
        1986, 8.4, {"Sigourney Weaver", "Newt"}, "Horror");

    std::list<Movie> database;
    database.insert(m1);
    database.insert(m2);


    std::cout << "hello world" << std::endl;
    /* code */
    return 0;
}
>>>>>>> eb4087acefb7e3964fbf7138371e79e42431d5e2
