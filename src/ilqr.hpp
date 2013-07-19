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

  public: //protected: /* Intermediate results, REVERSED TIME !!! */ 
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
    void initWithoutStiff (State_t state);
    
    State_t newStateInit ();

    void backwardLoop () {
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
    
    void forwardLoop () {

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
    };
 
    bool testConvergence (int compteur) {
      bool B = (compteur < 50);// && (tCost>0.1 || tCost<-0.1));
      return B;
 }

 void initBackwardLoop (const double r) {

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
 };

 void initForwardLoop (const double r) {

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
    };
 

   };

  template <typename Model>
  std::ostream& 
    operator<< (std::ostream& os, const Ilqr<Model>& m) { m.display(os); return os;}

    } // namespace viaopt

#include "viaopt/ilqr.t.cpp"

#endif // #ifndef __VIAOPT_ILQR__
