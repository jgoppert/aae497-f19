#include <fin.h>
#include <CasadiFunc.h>

CasadiFunc casadi_fins(fins_functions());

void setup() {
}

void loop() {
  double euler[3] = {1, 2, 3};
  double fins_pos[3];
  casadi_fins.arg(0, euler);
  casadi_fins.res(0, fins_pos);
  casadi_fins.eval();
}
