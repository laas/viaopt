#ifndef __VIAOPT_INTEGRATORRK4_tcpp__
#define __VIAOPT_INTEGRATORRK4_tcpp__

#include <Eigen/Core>
#include "viaopt/api.hpp"
#include <iostream>

namespace viaopt
{

  template<typename Model_t>
  IntegratorRK4<Model_t>::
  IntegratorRK4 (void)
  {
  }

  template<typename Model_t>
  void IntegratorRK4<Model_t>::
  display (std::ostream & os) const
  {
    os << "Dummy display: Fill me up!";
  }

  template<typename Model_t>
  void IntegratorRK4<Model_t>::
  setState (const State_t & state_)
  {
    state = state_;
  }

  template<typename Model_t>
  const typename Model_t::State_t& IntegratorRK4<Model_t>::
  getState ()
  {
    return state;
  }

  template<typename Model_t>
  typename Model_t::State_t& IntegratorRK4<Model_t>::
  integrate (const Control_t& control)
  {
    /*
      RK4 integrator:
      x_n+1 = x_n + f(x_n)*dt

      k1 = f(x)
      k2 = f(x+k1*dt/2)
      k3 = f(x+k2*dt/2)
      k4 = f(x+k3*dt)

      x = x  + (k1 + 2k2 + 2k3 + k4) / 6
     */

    State_t k1 = evolutionRK4 (state, control);
    State_t k2 = evolutionRK4 (state + k1*(Model_t::dT/2), control);
    State_t k3 = evolutionRK4 (state + k2*(Model_t::dT/2), control);
    State_t k4 = evolutionRK4 (state + k3*Model_t::dT/2, control);

    // std::cout<<"k1 "<<k1(2)<<" -  k2 : "<<k2(2)<<" -  k3 : "<<k3(2)<<" -  k4 : "<<k4(2)<< "\n";

    k1 += 2*k2;
    k1 += 2*k3;
    k1 += k4;
    state += k1*(1.0/6.0);

    //state = evolution(state,control);

    // std::cout <<k1(0) <<" "<<state(0)<< " \n"<<k1(1) <<" "<<state(1)<< " \n"<<k1(2) <<" "<<state(2)<< " \n"<<k1(3) <<" "<<state(3)<< " fin\n";
    return state;
  }

} // namespace viaopt

#endif // #ifndef __VIAOPT_INTEGRATOR_tcpp__

