#include <iostream>
#include <unordered_map>
#include <list>
#include <vector>
using namespace std;

class Movies
{
    private: 
    string m_title{""}; 
    string m_genre{"horror"};
    vector<string> m_actors;
    int m_year{1900};
    float m_rating{0.0};

    Movies(string title,int year,float rating,vector<string> actors,string genre):
        m_title(title), m_genre(genre), m_actors(actors), m_year(year), m_rating(rating)
    {
    }

    virtual ~Movies(){

    }
    void to_string(){
        if(m_genre.compare("horror"))
        {
            cout<<"The horror movie " << m_title << " is released in " << m_year << " starred by "
            << m_actors[0]<< " and " << m_actors[1] << " with a rating of " << m_rating <<endl;
        }
        else if(m_genre.compare("comedy"))
        {
            cout << m_title << " is a " << m_year<< " comedy with a " << m_rating <<
            " rating and starred " << m_actors[0] << " and "<<m_actors[1] <<endl;
        }
        else if(m_genre.compare("sci-fi"))
        {
            
        }
    }
};

int main(int argc, char const *argv[])
{
    
    return 0;
}
