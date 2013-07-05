#ifndef __VIAOPT_INTEGRATORRK4__
#define __VIAOPT_INTEGRATORRK4__

#include <Eigen/Core>
#include "viaopt/api.hpp"

namespace viaopt
{

  template<typename Model_t>
  class VIAOPT_EXPORT IntegratorRK4
    : protected Model_t
  {
  private:
 
  public: /* -- Types -- */
    typedef typename Model_t::State_t State_t;
    typedef typename Model_t::Control_t Control_t;

  public: //protected: /* Intermediate members */
    double timeStep;
    State_t state;

  public:
    IntegratorRK4 (void);
    void display (std::ostream & os) const;

  public:
    void setState (const State_t & state);
    const State_t& getState ();

    State_t& integrate ( const Control_t& control );

  };

  template <typename Model>
  std::ostream& 
  operator<< (std::ostream& os, const IntegratorRK4<Model>& m) { m.display(os); return os;}

} // namespace viaopt

#include "viaopt/integrator-rk4.t.cpp"

#endif // #ifndef __VIAOPT_INTEGRATOR__
