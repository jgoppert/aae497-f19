#include <string>
#include <list>
#include <iostream>

class Movie {
public:
    Movie(std::string title) : m_title(title) {}
    virtual ~Movie() {};
    std::string m_title{""};
};

int main(int argc, char const *argv[])
{
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
