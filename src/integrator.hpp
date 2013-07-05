#ifndef __VIAOPT_INTEGRATOR__
#define __VIAOPT_INTEGRATOR__

#include <Eigen/Core>
#include "viaopt/api.hpp"

namespace viaopt
{

  template<typename Model_t>
  class VIAOPT_EXPORT Integrator 
  {
  private:
 
  public: /* -- Types -- */
    typedef typename Model_t::State_t State_t;
    typedef typename Model_t::Control_t Control_t;

  public: //protected: /* Intermediate members */
    double timeStep;
    State_t state;

  public:
    Integrator (void);
    void display (std::ostream & os) const;

  public:
    void State_t& integrate ( const Control_t& control );

  };

  template <typename Model>
  std::ostream& 
  operator<< (std::ostream& os, const Integrator<Model>& m) { m.display(os); return os;}

} // namespace viaopt

#include "viaopt/integrator.t.cpp"

#endif // #ifndef __VIAOPT_INTEGRATOR__
