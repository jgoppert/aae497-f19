#include "casadi/CasadiFunc.hpp"
#include "func.h"
#include <iostream>

int main() {
	CasadiFunc two(f_functions());
	double x = 1;
	double y = 0;
	two.arg(0, &x);
	two.res(0, &y);
	two.eval();
	std::cout << "x: " << x << "y: " << y << std::endl;
	return 0;
}
