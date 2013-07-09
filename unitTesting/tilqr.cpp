/* -------------------------------------------------------------------------- *
 * 
 * Unittest of the Model classes.
 * 
 * -------------------------------------------------------------------------- */


#include "viaopt/model-awas.hpp"
#include "viaopt/ilqr.hpp"

using namespace viaopt;
int main (int , char** )
{
  Ilqr<ModelAwas> ilqr;

  ilqr.nbPreviewSteps = 5;
  ilqr.init();
  ilqr.computeControl( ModelAwas::State_t() );

}
