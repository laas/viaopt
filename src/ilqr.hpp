#ifndef __VIAOPT_ILQR__
#define __VIAOPT_ILQR__

#include <Eigen/Core>
#include <Eigen/LU>
#include <list>
#include "viaopt/api.hpp"
#include <iostream>

namespace viaopt
{

  template<typename Model_t>
  class VIAOPT_EXPORT Ilqr 
  {
  private:
 
  public: /* -- Types -- */
    typedef typename Model_t::State_t State_t; 
    typedef typename Model_t::Control_t Control_t;
    typedef typename Model_t::State_dx State_dx;
    typedef typename Model_t::State_du State_du;
    typedef typename Model_t::Cost_t Cost_t;
    typedef typename Model_t::Cost_dx Cost_dx; 
    typedef typename Model_t::Cost_dxx Cost_dxx;
    typedef typename Model_t::Cost_du Cost_du;
    typedef typename Model_t::Cost_duu Cost_duu;
    typedef typename Eigen::MatrixXd MatrixXd;
    typedef typename Eigen::VectorXd VectorXd;

    typedef std::list<VectorXd> StateList_t;
    typedef std::list<VectorXd> ControlList_t;
    typedef std::list<VectorXd> VxList ;
    typedef std::list<MatrixXd> VxxList ;
    typedef std::list<VectorXd> OpenLoopTermList ;
    typedef std::list<MatrixXd> GainList ;
   
    typedef std::list<VectorXd> InitState;
    typedef std::list<VectorXd> InitControl;
    typedef std::list<VectorXd> InitVx;
    typedef std::list<MatrixXd> InitVxx;
    typedef std::list<VectorXd> InitOpenLoop;
    typedef std::list<MatrixXd> InitGain;
    

  public: //protected: /* Parameters */
    Model_t model;
    int nbPreviewSteps;
    double timeStep;
    bool isInit;
    int compteur;

  public: //protected: /* Intermediate results, !!! REVERSED TIME !!! */ 
    StateList_t stateList;
    ControlList_t controlList;
    VxList vxList;
    VxxList vxxList;
    GainList gainList;
    OpenLoopTermList openLoopList;
    Cost_t costI;
    double tCost;
    
    // Initialization - without Stiffness
    InitState initState;
    InitControl initControl;
    InitVx initVx;
    InitVxx initVxx;
    InitOpenLoop initOpenLoop;
    InitGain initGain;

  public:
    Ilqr (void);
    void display (std::ostream & os) const;

  public:
    void init ();
    void computeControl (const State_t& state);

  public: // protected /* -- Intermediate functions -- */
    void newControlCycle (const State_t& state);

    void initListStateControl(State_t state,InitState initState, InitControl initControl);
    void initWithoutStiff    (State_t state);
    
    State_t newStateInit ();

    void backwardLoop    ();
    void forwardLoop     ();
    bool testConvergence (int compteur);

    void initBackwardLoop (const double r);
    void initForwardLoop  (const double r);
 
   };

  template <typename Model>
  std::ostream& operator<< (std::ostream& os, const Ilqr<Model>& m) { m.display(os); return os;}

} // namespace viaopt

#include "viaopt/ilqr.t.cpp"

#endif // #ifndef __VIAOPT_ILQR__
