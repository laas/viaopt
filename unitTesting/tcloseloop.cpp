/* -------------------------------------------------------------------------- *
 * 
 * Unittest of the Model classes.
 * 
 * -------------------------------------------------------------------------- */


#include "viaopt/model-awas.hpp"
#include "viaopt/ilqr.hpp"
#include "viaopt/integrator-rk4.hpp"

using namespace viaopt;
int main (int , char** )
{
  typedef ModelAwas::State_t State_t;
  typedef ModelAwas::Control_t Control_t;

  Ilqr<ModelAwas> ilqr;
  IntegratorRK4<ModelAwas> robot;

  robot.setState( State_t() );

  for (int cycle=0; cycle<100; ++cycle)
    {
      Control_t control = ilqr.computeControl (robot.getState());
      robot.integrate (control);
    }
}
