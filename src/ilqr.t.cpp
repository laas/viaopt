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
    S(3) = 0;

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
	*iter = C ; 
	if(iterControl!=initControl.end()){
	  iterControl++;
	}  
  }
  }

  template<typename Model_t>
  void Ilqr<Model_t>::
  initWithoutStiff (State_t state){
    assert( isInit );
    assert(state.size()==2);
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
    assert(state.size()==4);

    State_t stateWS(2);
    stateWS(0) = state(0);
    stateWS(1) = state(1);

    double r = state(2);

      initWithoutStiff(stateWS);
      for(int i=0;i<15;i++){
        initBackwardLoop(r);
         initForwardLoop(r);
      };

       model.print41(state,"init");

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
      // model.print41(state,"State");
      //  model.print41(Vx,"Vx");
      //   model.print44(Vxx,"Vxx");
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

      //  	model.print41(Vx,"Vx");
      //	model.print44(Vxx,"Vxx");
      //  	model.print41(state,"State");
      //	model.print21(control,"Control");

      State_dx fx = model.evolution_dx(state);
      State_du fu = model. evolution_du();

      Cost_dx Lx = model.integralCost_dx(state);
      Cost_dxx Lxx = model.integralCost_dxx(state);
      Cost_du Lu = model.integralCost_du(control);
      Cost_duu Luu = model.integralCost_duu();
	
      //    model.print41(Lx,"LX");
      //    model.print44(Lxx,"LXX");
      //   model.print21(Lu,"LU");
      //    model.print22(Luu,"Luu");

      //      	model.print44(fx,"fx");
      //    	model.print42(fu,"fu");

      for (int i=0;i<4;i++){
	Vxx(i,i) = Vxx(i,i)+ model.mu;}

      VectorXd Qx = Lx + fx.transpose()*Vx;
      VectorXd Qu = Lu + fu.transpose()*Vx;
      MatrixXd Qxx = Lxx + fx.transpose()*Vxx*fx;
      MatrixXd Quu = Luu + fu.transpose()*Vxx*fu;
      MatrixXd Qux = fu.transpose()*Vxx*fx;

      MatrixXd QuuInv = Quu.inverse();

      //        model.print41(Qx,"Qx");
      //  model.print22(QuuInv,"QuuInv");
      //        model.print44(Qxx,"Qxx");
      //       model.print22(Quu,"Quu");
      //       model.print24(Qux,"Qux");
      //       model.print21(Qu,"Qu");

      *iterOpenLoop =-(QuuInv*Qu);
      *iterGain = -(QuuInv*Qux);
      *iterVx = Qx - (Qux.transpose()*QuuInv*Qu);
      Vx = *iterVx;
      *iterVxx = Qxx - (Qux.transpose()*QuuInv*Qux);
      Vxx = *iterVxx;

      //  model.print24(Qux,"Qux");
      //   model.print22(QuuInv,"QuuInv");
      //   model.print21(Qu,"Qu");
      //   model.print41((Qux.transpose()*QuuInv*Qu),"1");
      //   model.print44((Qux.transpose()*QuuInv*Qux),"2");

      //      model.print24(*iterGain,"gain");
      //     model.print21(*iterOpenLoop,"k");
      //     model.print41(*iterVx,"Vx");
      //     model.print44(*iterVxx,"Vxx");



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

      //   model.print24(gain,"gain");
      //    model.print21(kopenLoop,"k");

      State_t dState = stateT-state;
      Control_t controlNew = control + model.alpha*kopenLoop + gain*dState;

      VectorXd vx = *iterVx;
      MatrixXd vxx = *iterVxx;     

      //    model.print41(vx,"VX"); 

      stateT = model.evolution(stateT,controlNew);
      iterState--; 
      state = *iterState;
      *iterState = stateT;
      *iterControl = controlNew;
      //    model.print41(state,"State");
	//  model.print21(controlNew,"ctrl");

  
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
    bool B = (compteur < 1 );// && (tCost>0.1 || tCost<-0.1));
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
      //   model.print21(state,"State");
      //   model.print21(Vx,"Vx");
      //   model.print22(Vxx,"Vxx");
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

      //    model.print21(state,"State");
      //    std::cout <<"control : "<<control<<"\n";

      State_dx fx = model.evolution_dxInit(r);
      State_du fu = model. evolution_duInit();

      Cost_dx Lx = model.integralCost_dxInit(state,r);
      Cost_dxx Lxx = model.integralCost_dxxInit(state,r);
      Cost_du Lu = model.integralCost_duInit(control);
      Cost_duu Luu = model.integralCost_duuInit();
	
      for (int i=0;i<2;i++){
	Vxx(i,i) = Vxx(i,i)+ 0.05;}

      VectorXd Qx = Lx + fx.transpose()*Vx;
      VectorXd Qu = fu.transpose()*Vx;
      MatrixXd Qxx = Lxx + fx.transpose()*Vxx*fx;
      MatrixXd Quu =  fu.transpose()*Vxx*fu;
      MatrixXd Qux = fu.transpose()*Vxx*fx;

      //  std::cout <<"fu : "<<fu<< "\n";

      //    std::cout <<"Qx : "<<Qx(0)<<" ; "<<Qx(1)<<"\n";
      //   std::cout <<"Qu : "<<Qu(0)<<"\n";
      //   model.print22(Qxx,"Qxx");
      //   std::cout <<"Quu : "<<Quu(0)<<"\n";
      //   std::cout <<"Qux : "<<Qux(0)<<" ; "<<Qux(1)<<"\n";

      MatrixXd QuuInv = Quu.inverse();

      *iterOpenLoop = -(QuuInv*Qu);
      *iterGain = -(QuuInv*Qux);
      *iterVx =Qx - (Qux.transpose()*QuuInv*Qu);
      Vx = *iterVx;
      *iterVxx = Qxx - (Qux.transpose()*QuuInv*Qux);
      Vxx = *iterVxx;

      //  model.print12(*iterGain,"gain");
      //     model.print21(*iterOpenLoop,"k");
      //     model.print21(*iterVx,"Vx");
      //     model.print22(*iterVxx,"Vxx");

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
     
      Control_t controlNew = control + 0.1*kopenLoop + gain*(stateT-state);

      std::cout<< "ctrl : " <<controlNew<<"\n";


      VectorXd vx = *iterVx;
      MatrixXd vxx = *iterVxx;      

      stateT = model.evolutionInit(stateT,controlNew,r);
      std::cout << "state : " <<stateT(0)<<" - "<<stateT(1)<<"\n";
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


