#ifndef __VIAOPT_ILQR_tcpp__
#define __VIAOPT_ILQR_tcpp__

#include <Eigen/Core>
#include <list>
#include "viaopt/api.hpp"

namespace viaopt
{

  template<typename Model_t>
  Ilqr<Model_t>::
  Ilqr (void)
  {
    init();
  }


  template<typename Model_t>
  void Ilqr<Model_t>::
  display (std::ostream & os) const
  {
    os << "Dummy display: Fill me up!";
  }

  template<typename Model_t>
  void Ilqr<Model_t>::
  init ()
  {
  }

  template<typename Model_t>
  typename Ilqr<Model_t>::Control_t Ilqr<Model_t>::
  computeControl (const State_t& state)
  {
    newControlCycle(state);
    for(;;)
      {
	backwardLoop();
	forwardLoop();
	if(testConvergence()) break;
      }

    return Control_t();
  }

} // namespace viaopt

#include "viaopt/ilqr.t.cpp"

#endif // #ifndef __VIAOPT_ILQR_tcpp__
