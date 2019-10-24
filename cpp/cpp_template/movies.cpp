#include <iostream>
#include <vector>
<<<<<<< HEAD

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
=======
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
>>>>>>> eb4087acefb7e3964fbf7138371e79e42431d5e2
    }
};

int main(int argc, char const *argv[])
{
<<<<<<< HEAD
    movie test1("movie",1975,9.5,{"A","B"},"Comedy");
    movie test2("drama",2019,5.5, {"C","D"},"history");
    std::cout << test1.m_title << " is made in " << test1.m_year << " with rating " << test1.m_rating << std::endl;
    std::cout << test2.m_title << " is made in " << test2.m_year << " with rating " << test2.m_rating << std::endl;

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
