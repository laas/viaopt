#ifndef __VIAOPT_ILQR_tcpp__
#define __VIAOPT_ILQR_tcpp__

#include <Eigen/Core>
#include <list>
#include "viaopt/api.hpp"
#include <iostream>

namespace viaopt
{

  template<typename Model_t>
  Ilqr<Model_t>::
  Ilqr (void)
    : nbPreviewSteps(0)
    ,isInit(false),
      compteur(0),
      costI(1)
  {
  }


  template<typename Model_t>
  void Ilqr<Model_t>::
  display (std::ostream & os) const
  {
    os << "Dummy display: Fill me up!";
  }

  template<typename Model_t>
  void Ilqr<Model_t>::
  init ()
  {
    assert( model.n>0);
    
    controlList.resize(model.n-1);
    stateList.resize(model.n);
    gainList.resize(model.n-1);
    vxList.resize(model.n);
    vxxList.resize(model.n);
    openLoopList.resize(model.n-1);

  initControl.resize(model.n-1);
    initState.resize(model.n);
    initGain.resize(model.n-1);
    initVx.resize(model.n);
    initVxx.resize(model.n);
    initOpenLoop.resize(model.n-1);

    isInit = true;
  }


  template<typename Model_t>
  void Ilqr<Model_t>::
  initListStateControl (State_t state,InitState initState,InitControl initControl){
    assert( isInit );
    InitState::iterator iterState = initState.begin();
    InitControl::iterator iterControl = initControl.begin();

    State_t S(4);
    State_t SW  = *iterState;
    S(0) = SW(0);
    S(1) = SW(1);
    S(2) = state(2);
    S(3) = state(3);

    for( StateList_t::iterator iter=stateList.begin();
	 stateList.end()!=iter;++iter )
      { 
	*iter = S ;
	if(iterState!=initState.end()){
	  iterState++;	
	  State_t SW  = *iterState;
	  S(0) = SW(0);
	  S(1) = SW(1);}
 }
    for( ControlList_t::iterator iter=controlList.begin();
	 controlList.end()!=iter;++iter )
      {
	Control_t C(2);
	Control_t CW = *iterControl;
	C(0) = CW(0);
	C(1) = 0.0; 
	*iter = C ;     }
  }

  template<typename Model_t>
  void Ilqr<Model_t>::
  initWithoutStiff (State_t state){
    assert( isInit );
    for( StateList_t::iterator iter=initState.begin();
	 initState.end()!=iter;++iter )
      { *iter = state ;     }
    for( ControlList_t::iterator iter=initControl.begin();
	 initControl.end()!=iter;++iter )
      { *iter = Eigen::VectorXd::Zero(1) ;     }
  }

  template<typename Model_t>
  void Ilqr<Model_t>::
  computeControl (const State_t& state)
  {
    assert(isInit);

    State_t stateWS(2);
    stateWS(0) = state(0);
    stateWS(1) = state(1);

    double r = state(2);

    initWithoutStiff(stateWS);
    for(int i=0;i<20;i++){
      initBackwardLoop(r);
      initForwardLoop(r);
    };

    initListStateControl(state,initState,initControl);
    backwardLoop();
    forwardLoop();
    testConvergence(compteur);
	while(testConvergence(compteur))
	  {
	    backwardLoop();
	    forwardLoop();
	  }
	compteur = 0;
  }
  
  template<typename Model_t>
  void Ilqr<Model_t>::
  newControlCycle (const State_t& stateCurrent)
  {
    assert(isInit);
    
    stateList.front() = stateCurrent;
    controlList.pop_front();
    controlList.push_back( Control_t() );
  }
  

  template <typename Model_t>
  typename Ilqr<Model_t>::State_t Ilqr<Model_t>::
  newStateInit ()
  {
      StateList_t::const_iterator iter = stateList.begin();
      std::advance(iter,model.n-model.window-1);
      //State_t state = *iter;
      return *iter;
    } 


} // namespace viaopt

#include "viaopt/ilqr.t.cpp"

#endif // #ifndef __VIAOPT_ILQR_tcpp__


