 /* -------------------------------------------------------------------------- *
 * 
 * Unittest of the Model classes.
 * 
 * -------------------------------------------------------------------------- */


#include "viaopt/model-awas.hpp"
#include "viaopt/ilqr.hpp"
#include <iostream>

using namespace viaopt;
int main (int , char** )
{
 
  Ilqr<ModelAwas> ilqr;
  ilqr.init();
  ModelAwas::State_t state = ModelAwas::initState(0.0,0.1);
  ModelAwas::Control_t control(2);
  control(0) = 0.15;
  control(1) = 0.4;

  double dT = 0.001;
  double inertia = 0.27;
  double stiffness = 10000.0;
  double Wx = 0.1;
  double Wlim = 0.01;
  double Tdes = 50;
  double Wu0 = 0.1;
  double Wu1 = 0.001;
  

  // std::cout<<cost_du(0)<<"\n"<<cost_du(1)<<"\n";//<<cost_dx(2)<<"\n"<<cost_dx(3)<<"\n";
  // std::cout << State(0,0)<<" " << State(0,1)<<" " << State(0,2)<<" " << State(0,3)<<"\n";
  // std::cout << State(1,0)<<" " << State(1,1)<<" " << State(1,2)<<" " << State(1,3)<<"\n";  
  // std::cout << State(2,0)<<" " << State(2,1)<<" " << State(2,2)<<" " << State(2,3)<<"\n";
  // std::cout << State(3,0)<<" " << State(3,1)<<" " << State(3,2)<<" " << State(3,3)<<"\n";

   std::cout <<"\n"<<  control(0)<<"\n"<<control(1)<<"\n";

  // ilqr.computeControl(state);


}
