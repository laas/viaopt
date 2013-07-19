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
 
  public : //protected:
    double stiffness;
    double inertia;
    double Wx;
    double Wu0;
    double Wu1;
    double Wterminal;
    double Wlim;
    double dT;
    double Tdes;
    double mu;
    double alpha;
    int n; //size of the vector during optimization
    double window;
    
  public:
    ModelAwas (void);
    ~ModelAwas (void);
    void display (std::ostream & os) const;

  public:

    static State_t initState(const double theta, const double r);

    virtual State_t evolution       (const State_t& state, const Control_t& control) const;
    virtual State_dx evolution_dx   (const State_t& state) const;
    virtual State_du evolution_du   () const;
    virtual State_t  evolutionRK4   (const State_t& state,const Control_t& control) const;
  
    virtual Cost_t integralCost     (const State_t& state, const Control_t& control) const;
    virtual Cost_dx integralCost_dx (const State_t& state) const;
    virtual Cost_du integralCost_du (const Control_t& control) const;
    virtual Cost_dxx integralCost_dxx (const State_t& state) const;
    virtual Cost_duu integralCost_duu () const;
 
    virtual Cost_t terminalCost     (const State_t& state) const;
    virtual Cost_dx terminalCost_dx (const State_t& state) const;
    virtual Cost_dxx terminalCost_dxx (const State_t& state) const;

    virtual void Print41 (const Cost_dx& C,const std::string S) const;
    virtual void Print22 (const Cost_duu& C,const std::string S) const; 
    virtual void Print44 (const Cost_dxx& C,const std::string S) const;
    virtual void Print21 (const Cost_du& C,const std::string S) const;
    virtual void Print24 (const Cost_dxx C,const std::string S) const;
    virtual void Print42 (const Cost_dxx C,const std::string S) const;

    // Initialization
    virtual State_t evolutionInit (const State_t& state, const Control_t& control,const double r) const;
    virtual State_dx evolution_dxInit   (const double r) const;
    virtual State_du evolution_duInit () const;
    virtual Cost_dx integralCost_dxInit (const State_t& state,const double r) const;
    virtual Cost_du integralCost_duInit (const Control_t& control) const;
    virtual Cost_dxx integralCost_dxxInit (const State_t& state,const double r) const;
    virtual Cost_duu integralCost_duuInit () const;
  };

  inline std::ostream& 
  operator<< (std::ostream& os, const ModelAwas& m) { m.display(os); return os;}

} // namespace viaopt


#endif // #ifndef __VIAOPT_MODELAWAS__

