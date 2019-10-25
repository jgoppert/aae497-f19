#include <iostream>

template <class Scalar>
Scalar divide_by_two(Scalar x) {
    return x/2;
}


int main(int argc, char const *argv[])
{
    double res1 = divide_by_two<double>(3);
    double res2 = divide_by_two<int>(3);
    std::cout << "res1: " << res1 << " res2: " << res2 << std::endl;
    return 0;
}
