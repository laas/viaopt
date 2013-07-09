#ifndef __VIAOPT_ILQR__
#define __VIAOPT_ILQR__

#include <Eigen/Core>
#include <list>
#include "viaopt/api.hpp"

namespace viaopt
{

  template<typename Model_t>
  class VIAOPT_EXPORT Ilqr 
  {
  private:
 
  public: /* -- Types -- */
    typedef typename Model_t::State_t State_t;
    typedef typename Model_t::Control_t Control_t;
    typedef std::list<State_t> StateList_t;
    typedef std::list<Control_t> ControlList_t;

  public: //protected: /* Parameters */
    int nbPreviewSteps;
    double timeStep;
    bool isInit;

  public: //protected: /* Intermediate results */
    StateList_t stateList;
    ControlList_t controlList;

  public:
    Ilqr (void);
    void display (std::ostream & os) const;

  public:
    void init ();
    Control_t computeControl (const State_t& state);

  public: // protected /* -- Intermediate functions -- */
    void newControlCycle (const State_t& state);
    void backwardLoop () { /* TODO */ }
    void forwardLoop () { /* TODO */ }
    bool testConvergence () { /* TODO */ }

  };

  template <typename Model>
  std::ostream& 
  operator<< (std::ostream& os, const Ilqr<Model>& m) { m.display(os); return os;}

} // namespace viaopt

#include "viaopt/ilqr.t.cpp"

#endif // #ifndef __VIAOPT_ILQR__
