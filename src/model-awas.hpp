#ifndef __VIAOPT_MODELAWAS__
#define __VIAOPT_MODELAWAS__

#include <Eigen/Core>
#include "viaopt/api.hpp"
// #include "viaopt/model-abstract.hpp"

namespace viaopt
{

  class VIAOPT_EXPORT ModelAwas 
  {
  private:

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

 
  public : //protected:
    double stiffness;
    double inertia;
    double Wx; // weight in the cost function 
    double Wu0; // idem
    double Wu1; //idem
    double Wterminal; //idem
    double Wlim; // idem
    double dT; 
    double Tdes; // desired torque
    double Cible; // desired Position (useless for the moment)
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

    State_t  evolution      (const State_t& state, const Control_t& control) const;
    State_dx evolution_dx   (const State_t& state) const;
    State_du evolution_du   () const;
    State_t  evolutionRK4   (const State_t& state,const Control_t& control) const;
  
    Cost_t   integralCost     (const State_t& state, const Control_t& control) const;
    Cost_dx  integralCost_dx  (const State_t& state) const;
    Cost_du  integralCost_du  (const Control_t& control) const;
    Cost_dxx integralCost_dxx (const State_t& state) const;
    Cost_duu integralCost_duu () const;
 
    Cost_t   terminalCost     (const State_t& state) const;
    Cost_dx  terminalCost_dx  (const State_t& state) const;
    Cost_dxx terminalCost_dxx (const State_t& state) const;

    void print41 (const Cost_dx& C,const std::string S) const;
    void print22 (const Cost_duu& C,const std::string S) const; 
    void print44 (const Cost_dxx& C,const std::string S) const;
    void print21 (const Cost_du& C,const std::string S) const;
    void print24 (const Cost_dxx C,const std::string S) const;
    void print42 (const Cost_dxx C,const std::string S) const;

    // Initialization
    State_t  evolutionInit        (const State_t& state, const Control_t& control,const double r) const;
    State_dx evolution_dxInit     (const double r) const;
    State_du evolution_duInit     () const;
    Cost_dx  integralCost_dxInit  (const State_t& state,const double r) const;
    Cost_du  integralCost_duInit  (const Control_t& control) const;
    Cost_dxx integralCost_dxxInit (const State_t& state,const double r) const;
    Cost_duu integralCost_duuInit () const;
  };

  inline std::ostream& 
  operator<< (std::ostream& os, const ModelAwas& m) { m.display(os); return os;}

} // namespace viaopt


#endif // #ifndef __VIAOPT_MODELAWAS__

