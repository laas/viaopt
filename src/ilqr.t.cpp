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
    :
    nbPreviewSteps(0)
    ,isInit(false)
    ,compteur(0)
    ,costI(1)
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


  template <typename Model_t>
  void Ilqr<Model_t>::backwardLoop ()
  {
    Cost_dx Vx;
    Cost_dxx Vxx;
    {
      // Final State
      const State_t& state = stateList.front(); 
      Vx =  model.terminalCost_dx(state);
      vxList.front() = Vx;
      Vxx = model.terminalCost_dxx(state);
      vxxList.front() = Vxx;
    }
 
    StateList_t::iterator iterState = stateList.begin();
    VxList::iterator iterVx = vxList.begin(); 
    VxxList::iterator iterVxx = vxxList.begin();
    OpenLoopTermList::iterator iterOpenLoop = openLoopList.begin();
    GainList::iterator iterGain = gainList.begin();

    std::advance(iterState,1);
    std::advance(iterVx,1);
    std::advance(iterVxx,1);

    for (ControlList_t::iterator iterControl=controlList.begin(); iterControl!=controlList.end();++iterControl){

      const State_t & state = *iterState;
      const Control_t & control = *iterControl;

      //	model.Print41(Vx,"Vx");
      //	model.Print44(Vxx,"Vxx");
      //	model.Print41(state,"State");
      //	model.Print21(control,"Control");

      State_dx fx = model.evolution_dx(state);
      State_du fu = model. evolution_du();

      Cost_dx Lx = model.integralCost_dx(state);
      Cost_dxx Lxx = model.integralCost_dxx(state);
      Cost_du Lu = model.integralCost_du(control);
      Cost_duu Luu = model.integralCost_duu();
	
      //	model.Print41(Lx,"LX");
      //	model.Print44(Lxx,"LXX");
      //	model.Print21(Lu,"LU");
      //	model.Print22(Luu,"Luu");

      //	model.Print44(fx,"fx");
      //	model.Print42(fu,"fu");

      for (int i=0;i<4;i++){
	Vxx(i,i) = Vxx(i,i)+ model.mu;}

      VectorXd Qx = Lx + fx.transpose()*Vx;
      VectorXd Qu = Lu + fu.transpose()*Vx;
      MatrixXd Qxx = Lxx + fx.transpose()*Vxx*fx;
      MatrixXd Quu = Luu + fu.transpose()*Vxx*fu;
      MatrixXd Qux; Qux = fu.transpose()*Vxx*fx;

      MatrixXd QuuInv = Quu.inverse();
      Quu(0,1) =  Quu(0,1);
      Quu(1,0) =  Quu(1,0);

      //	model.Print22(Quu,"Quu");

      *iterOpenLoop =-(QuuInv*Qu);
      *iterGain = -(QuuInv*Qux);
      *iterVx = Qx - (Qux.transpose()*QuuInv*Qu);
      *iterVxx = Qxx - (Qux.transpose()*QuuInv*Qux);

      if ( iterState!=stateList.end()) {
	iterState++; 
	iterGain++;
	iterOpenLoop++;
	++iterVx;
	iterVxx++;
      }
    }
  }
    
  template <typename Model_t>
  void Ilqr<Model_t>:: forwardLoop ()
  {
    StateList_t::iterator iterState = stateList.end(); iterState--;
    VxList::iterator iterVx = vxList.end();  iterVx--;
    VxxList::iterator iterVxx = vxxList.end(); iterVxx--;
    OpenLoopTermList::iterator iterOpenLoop = openLoopList.end(); iterOpenLoop--;
    GainList::iterator iterGain = gainList.end(); iterGain--;

    State_t stateT = *iterState;
    State_t state = *iterState;
    Cost_t cost= 0.0;
      
    for (ControlList_t::reverse_iterator iterControl=controlList.rbegin(); iterControl!=controlList.rend();++iterControl){

      Control_t control = *iterControl;
      MatrixXd gain = *iterGain;
      VectorXd kopenLoop = *iterOpenLoop;

      State_t dState = stateT-state;
      Control_t controlNew = control + model.alpha*kopenLoop + gain*dState;
      controlNew(1) = controlNew(1);

      VectorXd vx = *iterVx;
      MatrixXd vxx = *iterVxx;      

      stateT = model.evolution(stateT,controlNew);
      iterState--; 
      state = *iterState;
      *iterState = stateT;
      *iterControl = controlNew;
  
      if (iterGain!=gainList.begin())
	{iterGain--;
	  iterOpenLoop--;
	  iterVx--;
	  iterVxx--;}
    }

    for (StateList_t::iterator iter=stateList.begin(); iter!=stateList.end();++iter){
      State_t S1 = *iter;
    }

    tCost = (costI-cost)/costI;
    costI = cost;
    compteur++;
  }
 
  template <typename Model_t>
  bool Ilqr<Model_t>:: testConvergence (int compteur)
  {
    bool B = (compteur < 50);// && (tCost>0.1 || tCost<-0.1));
    return B;
  }

  template <typename Model_t>
  void Ilqr<Model_t>::initBackwardLoop (const double r)
  {

    Cost_dx Vx;
    Cost_dxx Vxx;
    {
      // Final State
      const State_t& state = initState.front(); 
      Vx = 1000*model.integralCost_dxInit(state,r);
      vxList.front() = Vx;
      Vxx = 1000*model.integralCost_dxxInit(state,r);
      vxxList.front() = Vxx;
    }
 
    InitState::iterator iterState = initState.begin();
    InitVx::iterator iterVx = initVx.begin(); 
    InitVxx::iterator iterVxx = initVxx.begin();
    InitOpenLoop::iterator iterOpenLoop = initOpenLoop.begin();
    InitGain::iterator iterGain = initGain.begin();

    std::advance(iterState,1);
    std::advance(iterVx,1);
    std::advance(iterVxx,1);

    for (InitControl::iterator iterControl=initControl.begin(); iterControl!=initControl.end();++iterControl){

      const State_t & state = *iterState;
      const Control_t & control = *iterControl;

      State_dx fx = model.evolution_dxInit(r);
      State_du fu = model. evolution_duInit();

      Cost_dx Lx = model.integralCost_dxInit(state,r);
      Cost_dxx Lxx = model.integralCost_dxxInit(state,r);
      Cost_du Lu = model.integralCost_duInit(control);
      Cost_duu Luu = model.integralCost_duuInit();
	
      for (int i=0;i<2;i++){
	Vxx(i,i) = Vxx(i,i)+ model.mu;}

      VectorXd Qx = Lx + fx.transpose()*Vx;
      VectorXd Qu = Lu + fu.transpose()*Vx;
      MatrixXd Qxx = Lxx + fx.transpose()*Vxx*fx;
      MatrixXd Quu = Luu + fu.transpose()*Vxx*fu;
      MatrixXd Qux; Qux = fu.transpose()*Vxx*fx;

      MatrixXd QuuInv = Quu.inverse();

      *iterOpenLoop = -(QuuInv*Qu);
      *iterGain = -(QuuInv*Qux);
      *iterVx =Qx - (Qux.transpose()*QuuInv*Qu);
      *iterVxx = Qxx - (Qux.transpose()*QuuInv*Qux);

      if ( iterState!=stateList.end()) {
	iterState++; 
	iterGain++;
	iterOpenLoop++;
	++iterVx;
	iterVxx++;
      }
    }
  }

  template <typename Model_t>
  void Ilqr<Model_t>::initForwardLoop (const double r)
  {

    InitState::iterator iterState = initState.end(); iterState--;
    InitVx::iterator iterVx = initVx.end();  iterVx--;
    InitVxx::iterator iterVxx = initVxx.end(); iterVxx--;
    InitOpenLoop::iterator iterOpenLoop =initOpenLoop.end(); iterOpenLoop--;
    InitGain::iterator iterGain = initGain.end(); iterGain--;

    State_t stateT = *iterState;
    State_t state = *iterState;
       
    for (InitControl::reverse_iterator iterControl=initControl.rbegin(); iterControl!=initControl.rend();++iterControl){

      Control_t control = *iterControl;
      MatrixXd gain = *iterGain;
      VectorXd kopenLoop = *iterOpenLoop;
     
      Control_t controlNew = control + model.alpha*kopenLoop + gain*(stateT-state);

      VectorXd vx = *iterVx;
      MatrixXd vxx = *iterVxx;      

      stateT = model.evolutionInit(stateT,controlNew,r);
      ;
      iterState--; 
      state = *iterState;
      *iterState = stateT;
      *iterControl = controlNew;
      
      if (iterGain!=gainList.begin())
	{iterGain--;
	  iterOpenLoop--;
	  iterVx--;
	  iterVxx--;}
    }
  }
 


} // namespace viaopt

#include "viaopt/ilqr.t.cpp"

#endif // #ifndef __VIAOPT_ILQR_tcpp__


