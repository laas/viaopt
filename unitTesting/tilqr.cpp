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
  assert(state.size()==4);
  ModelAwas::Control_t control(2);
  control(0) = 0.15;
  control(1) = 0.4;
  assert(control.size()==2);


  double dT = 0.001;
  double inertia = 0.27;
  double stiffness = 10000.0;
  double Wx = 0.1;
  double Wlim = 0.01;
  double Tdes = 50;
  double Wu0 = 0.1;
  double Wu1 = 0.001;
  



}
