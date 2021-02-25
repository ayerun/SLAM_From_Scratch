#include<random>
#include<iostream>

std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}

int main()
{
    double n = 0;
    std::normal_distribution<> d(1, 1);
    // d(get_random());
    std::cout << d(get_random()) << std::endl;
    std::cout << d << std::endl;
}