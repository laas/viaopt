#ifndef __VIAOPT_MODELABSTRACT__
#define __VIAOPT_MODELABSTRACT__

#include <Eigen/Core>
#include "viaopt/api.hpp"

namespace viaopt
{

  class VIAOPT_EXPORT ModelAbstract
  {
  private:
 
  protected:
 
  public:
    typedef Eigen::VectorXd State_t;
    typedef Eigen::VectorXd Control_t;
    typedef double Cost_t;

    typedef Eigen::MatrixXd State_dx;
    typedef Eigen::MatrixXd State_du;
    typedef Eigen::VectorXd Cost_dx;
    typedef Eigen::VectorXd Cost_du;

  public:
    ModelAbstract (void);
    void display (std::ostream & os) const;

    virtual State_t
      evolution       (const State_t& state, const Control_t& control) const = 0;
    virtual State_dx
      evolution_dx    (const State_t& state, const Control_t& control) const = 0;
    virtual State_du
      evolution_du    (const State_t& state, const Control_t& control) const = 0;

    virtual Cost_t
      integralCost    (const State_t& state, const Control_t& control) const = 0;
    virtual Cost_dx
      integralCost_dx (const State_t& state, const Control_t& control) const = 0;
    virtual Cost_du
      integralCost_du (const State_t& state, const Control_t& control) const = 0;

    virtual Cost_t
      terminalCost    (const State_t& state, const Control_t& control) const = 0;
    virtual Cost_dx
      terminalCost_dx (const State_t& state, const Control_t& control) const = 0;

  };

  inline std::ostream& 
  operator<< (std::ostream& os, const ModelAbstract& m) { m.display(os); return os;}

} // namespace viaopt


#endif // #ifndef __VIAOPT_MODELABSTRACT__
