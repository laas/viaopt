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
    typedef Eigen::MatrixXd Cost_dxx;
    typedef Eigen::MatrixXd Cost_duu;
    //   typedef Eigen::MatrixXd Cost_dxu;
    //   typedef Eigen::MatrixXd Cost_dux;

  public:
    ModelAbstract (void);
    void display (std::ostream & os) const;

    virtual State_t

      evolution       (const State_t& state, const Control_t& control) const = 0;
    virtual State_dx
      evolution_dx    (const State_t& state) const = 0;
    virtual State_du
      evolution_du    () const = 0;

    virtual Cost_t
      integralCost    (const State_t& state, const Control_t& control) const = 0;
    virtual Cost_dx
      integralCost_dx (const State_t& state) const = 0;
    virtual Cost_du
      integralCost_du (const Control_t& control) const = 0;
    virtual Cost_dxx
      integralCost_dxx (const State_t& state) const = 0;
    virtual Cost_duu
      integralCost_duu () const = 0;

    virtual Cost_t
      terminalCost    (const State_t& state) const = 0;
    virtual Cost_dx
      terminalCost_dx (const State_t& state) const = 0;
    virtual Cost_dxx
      terminalCost_dxx (const State_t& state) const = 0;
 

  // Initialization

    virtual State_t  evolutionInit     (const State_t& state, const Control_t& control,const double r) const;
    virtual State_dx  evolution_dxInit   (const double r) const;
    virtual State_du evolution_duInit   () const;
    virtual Cost_dx integralCost_dxInit (const State_t& state,const double r) const;

    };

  inline std::ostream& 
  operator<< (std::ostream& os, const ModelAbstract& m) { m.display(os); return os;}

} // namespace viaopt


#endif // #ifndef __VIAOPT_MODELABSTRACT__
