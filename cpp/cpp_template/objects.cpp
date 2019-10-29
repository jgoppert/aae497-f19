#include <iostream>
#include <vector>
#include <memory>

class Animal {
public:
    Animal() {
        std::cout << "making an animal" << std::endl;
    }
    virtual ~Animal() {
        std::cout << "destroying an animal" << std::endl;
    }
    void talk() {};
};

class Dog: public Animal {
public:
    Dog() {
        std::cout << "--making a dog" << std::endl;
    }
    virtual ~Dog() {
        std::cout << "--destroying a dog" << std::endl;
    }
    virtual void talk() {
        std::cout << "--bark" << std::endl;
    }
};

class Cat: public Animal {
public:
    Cat() {
        std::cout << "--making a cat" << std::endl;
    }
    virtual ~Cat() {
        std::cout << "--destroying a cat" << std::endl;
    }
    virtual void talk() {
        std::cout << "--meow" << std::endl;
    }
};


int main(int argc, char const *argv[])
{
     {
        std::cout << "\ntest 1" << std::endl;
        std::vector<Animal *> myAnimals;
        Animal * dog = new Dog();
        Animal * cat = new Cat();
        myAnimals.push_back(dog);
        myAnimals.push_back(cat);
        // there is a memory leak here, since the delete is never called
        std::cout << "MEMORY LEAK" << std::endl;
     }
     {
        std::cout << "\ntest 2" << std::endl;
        std::vector<Animal *> myAnimals;
        Animal * dog = new Dog();
        myAnimals.push_back(dog);
        myAnimals.push_back(new Cat()); // you can call new within push back and rely on conversion to Animal *
        for (auto animal : myAnimals) {
            delete animal;
        }
     }
     {
        std::cout << "\ntest 3" << std::endl;
        typedef std::shared_ptr<Animal> AnimalPtr;
        std::vector<AnimalPtr> myAnimals;
        std::shared_ptr<Dog> dog = std::make_shared<Dog>();
        auto cat = std::make_shared<Dog>(); // you can use auto to save typing
        myAnimals.push_back(dog);
        myAnimals.push_back(cat);
     }
    /* code */
    return 0;
}
