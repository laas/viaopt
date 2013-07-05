#ifndef __VIAOPT_MODELAWAS__
#define __VIAOPT_MODELAWAS__

#include <Eigen/Core>
#include "viaopt/api.hpp"
#include "viaopt/model-abstract.hpp"

namespace viaopt
{

  class VIAOPT_EXPORT ModelAwas 
    :
    public ModelAbstract
  {
  private:
 
  protected:
 
  public:
    ModelAwas (void);
    ~ModelAwas (void);
    void display (std::ostream & os) const;

  public:

    State_t evolution       (const State_t& state, const Control_t& control) const;
    State_dx evolution_dx   (const State_t& state, const Control_t& control) const;
    State_du evolution_du   (const State_t& state, const Control_t& control) const;

    Cost_t integralCost     (const State_t& state, const Control_t& control) const;
    Cost_dx integralCost_dx (const State_t& state, const Control_t& control) const;
    Cost_du integralCost_du (const State_t& state, const Control_t& control) const;

    Cost_t terminalCost     (const State_t& state, const Control_t& control) const;
    Cost_dx terminalCost_dx (const State_t& state, const Control_t& control) const;

  };

  inline std::ostream& 
  operator<< (std::ostream& os, const ModelAwas& m) { m.display(os); return os;}

} // namespace viaopt


#endif // #ifndef __VIAOPT_MODELAWAS__
