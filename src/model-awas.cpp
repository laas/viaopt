#include "viaopt/model-awas.hpp"

namespace viaopt
{

  /* --- CONSTRUCTOR -------------------------------------------------------- */ 
  ModelAwas::
  ModelAwas ()
  {}

  ModelAwas::
  ~ModelAwas ()
  {}

  /* --- DISPLAY ------------------------------------------------------------ */
  void ModelAwas::
  display (std::ostream& os) const
  {
    os << "Dummy display, fill me up!";
  }

  /* --- DERIVATION FUNCTIONS ----------------------------------------------- */
  ModelAwas::State_t ModelAwas::
  evolution       (const State_t& state, const Control_t& control) const
  {
    return State_t();
  }

  ModelAwas::State_dx ModelAwas::
  evolution_dx   (const State_t& state, const Control_t& control) const
  {
    return State_dx();
  }

  ModelAwas::State_du ModelAwas::
  evolution_du   (const State_t& state, const Control_t& control) const
  {
    return State_du();
  }

  ModelAwas::Cost_t ModelAwas::
  integralCost     (const State_t& state, const Control_t& control) const
  {
    return Cost_t();
  }

  ModelAwas::Cost_dx ModelAwas::
  integralCost_dx (const State_t& state, const Control_t& control) const
  {
    return Cost_dx();
  }

  ModelAwas::Cost_du ModelAwas::
  integralCost_du (const State_t& state, const Control_t& control) const
  {
    return Cost_du();
  }

  ModelAwas::Cost_t ModelAwas::
  terminalCost     (const State_t& state, const Control_t& control) const
  {
      return Cost_t();
  }

  ModelAwas::Cost_dx ModelAwas::
  terminalCost_dx (const State_t& state, const Control_t& control) const
  {
    return Cost_dx();
  }




} // namespace viaopt
