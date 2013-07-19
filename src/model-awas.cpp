#include "viaopt/model-awas.hpp"
#include <iostream>

namespace viaopt
{

  /* --- CONSTRUCTOR -------------------------------------------------------- */ 
  ModelAwas::
  ModelAwas ()
    :
    stiffness(10000.0),
    inertia(0.27),
    Wx(0.1),
    Wu0(0.100),
    Wu1(0.0010),
    Wterminal(1000),
    Wlim(0.010),
    dT(0.0001),
    Tdes(50),
    mu(0.005),
    alpha(0.001),
    n(14
),
    window(5)

  {
}

  ModelAwas::
  ~ModelAwas ()
  {}

  /* --- DISPLAY ------------------------------------------------------------ */
  void ModelAwas::
  display (std::ostream& os) const
  {
    os << "Dummy display, fill me up!";
  }

  /* --- INITIALIZATION ----------------------------------------------------- */
  ModelAwas::State_t ModelAwas::initState (const double theta,const double r) {
    State_t state(4);
    state(0) = theta;
    state(2) = r;

    return state;
 }

  /* --- DERIVATION FUNCTIONS ----------------------------------------------- */
  ModelAwas::State_t ModelAwas::
  evolution       (const State_t& state, const Control_t& control) const
  {
    const double theta = state(0)+dT*state(1);
    const double r = state(2)+dT*state(3);
    const double rp = state(3)+dT*control(1);
    const double thetap = state(1)+dT*(-stiffness/inertia*state(2)*state(2)*state(0)-control(0));

    State_t state_new(4);
    state_new(0) = theta;
    state_new(1) = thetap;
    state_new(2) = r;
    state_new(3) = rp;    

    return state_new;
  }

  ModelAwas::State_dx ModelAwas::
  evolution_dx   (const State_t& state) const
  {
    State_dx state_dx(4,4);
    state_dx(0,0) = 1;
    state_dx(0,1) = dT;
    state_dx(1,0) = -dT*stiffness/inertia*state(2)*state(2);
    state_dx(1,1) = 1;
    state_dx(1,2) = -2*dT*stiffness/inertia*state(0)*state(2);
    state_dx(2,2) = 1;
    state_dx(2,3) = dT;
    state_dx(3,3) = 1;

    state_dx(0,2) = 0 ;
    state_dx(0,3) = 0;
    state_dx(1,3) = 0;
    state_dx(2,0) = 0;
    state_dx(2,1) = 0;
    state_dx(3,0) = 0;
    state_dx(3,1) = 0;
    state_dx(3,2) = 0;


    return state_dx;
  }

  ModelAwas::State_t ModelAwas::
  evolutionRK4   (const State_t& state,const Control_t& control) const
  {
    State_t stateNew(4);
    stateNew(0) = state(1);
    stateNew(2) = state(3);
    stateNew(3) = control(1);
    stateNew(1) = (-stiffness/inertia*state(2)*state(2)*state(0)-control(0));
    return stateNew;
  }



  ModelAwas::State_du ModelAwas::
  evolution_du   () const
  {
    State_du state_du(4,2);
    state_du(1,0) = -dT;
    state_du(3,1) = dT;

    state_du(0,0) = 0;
    state_du(0,1) = 0;
    state_du(1,1) = 0;
    state_du(2,0) = 0;
    state_du(2,1) = 0;
    state_du(3,0) = 0;

    return state_du;
  }


  ModelAwas::Cost_t ModelAwas::
  integralCost     (const State_t& state, const Control_t& control) const
  {
   Cost_t cost;
   cost = Wterminal*(stiffness*state(2)*state(2)*state(0)-Tdes)*(stiffness*state(2)*state(2)*state(0)-Tdes) + Wu0*control(0)*control(0) + Wu1*control(1)*control(1) + Wlim *(-log(state(2)-0.05)-log(0.15-state(2)));

      return cost;
  }

   ModelAwas::Cost_dx ModelAwas::
  integralCost_dx (const State_t& state) const
  {
    Cost_dx cost_dx(4);
    cost_dx(0) = Wx*2*stiffness*state(2)*state(2)*(stiffness*state(2)*state(2)*state(0)-Tdes);
    cost_dx(1) = 0.0;
    cost_dx(3) = 0.0;
    cost_dx(2) = Wx*4*stiffness*state(0)*state(2)*(stiffness*state(2)*state(2)*state(0)-Tdes) + Wlim*(-1/(state(2)-0.05)-1/(0.15-state(2)));
    return cost_dx;
  }

  ModelAwas::Cost_du ModelAwas::
  integralCost_du (const Control_t& control) const
  {
    Cost_du cost_du(2);
    cost_du(0) = 2*Wu0*control(0);
    cost_du(1) = 2*Wu1*control(1);
    return cost_du;
  }

  ModelAwas::Cost_dxx ModelAwas::
  integralCost_dxx (const State_t& state) const
  {
    Cost_dxx cost_dxx(4,4);
    cost_dxx(0,0) = Wx*2*stiffness*stiffness*state(2)*state(2)*state(2)*state(2);
    cost_dxx(0,2) = Wx*4*stiffness*state(2)*(2*stiffness*state(2)*state(2)*state(0)-Tdes);
    cost_dxx(2,0) = cost_dxx(0,2);
    cost_dxx(2,2) = Wx*(12*stiffness*stiffness*state(2)*state(2)*state(0)*state(0)-4*stiffness*state(0)*Tdes)+ Wlim*(+1/(state(2)-0.05)*1/(state(2)-0.05)-1/(0.15-state(2))/(0.15-state(2))); 
    cost_dxx(0,1) =0;
    cost_dxx(0,3) =0;
    cost_dxx(1,0) =0;
    cost_dxx(1,1) =0;
    cost_dxx(1,2) =0;
    cost_dxx(1,3) =0;
    cost_dxx(2,1) =0;
    cost_dxx(2,3) =0;
    cost_dxx(3,0) =0;
    cost_dxx(3,1) =0;
    cost_dxx(3,2) =0;
    cost_dxx(3,3) =0;


    return cost_dxx;
  }

 ModelAwas::Cost_duu ModelAwas::
  integralCost_duu () const
  {
    Cost_duu cost_duu(2,2);
    cost_duu(0,0) = 2*Wu0;
    cost_duu(1,1) = 2*Wu1;

    cost_duu(0,1) = 0;
    cost_duu(1,0) = 0;

    return cost_duu;
  }

  ModelAwas::Cost_t ModelAwas::
  terminalCost     (const State_t& state) const
  {
    return  Wterminal*(Wx*(stiffness*state(2)*state(2)*state(0)-Tdes)*(stiffness*state(2)*state(2)*state(0)-Tdes) + Wlim *(-log(state(2)-0.05)-log(0.15-state(2))));

  }

  ModelAwas::Cost_dx ModelAwas::
  terminalCost_dx (const State_t& state) const
  { 
    return Wterminal*integralCost_dx(state);
  }

  ModelAwas::Cost_dxx ModelAwas::
  terminalCost_dxx (const State_t& state) const
  { 
    return Wterminal*integralCost_dxx(state);
  }

  // Print Matrix and Vectors

  void ModelAwas::Print44 (const Cost_dxx& C,const std::string S) const
  { 
    std::cout<<"\n"<<S<<"\n";
    std::cout<<C(0,0)<<"\t"<<C(0,1)<<"\t"<<C(0,2)<<"\t"<<C(0,3)<<"\n";
    std::cout<<C(1,0)<<"\t"<<C(1,1)<<"\t"<<C(1,2)<<"\t"<<C(1,3)<<"\n";
    std::cout<<C(2,0)<<"\t"<<C(2,1)<<"\t"<<C(2,2)<<"\t"<<C(2,3)<<"\n";
    std::cout<<C(3,0)<<"\t"<<C(3,1)<<"\t"<<C(3,2)<<"\t"<<C(3,3)<<"\n";
  }

 void ModelAwas::Print22 (const Cost_duu& C,const std::string S) const
  { 
    std::cout<<"\n"<<S<<"\n";
    std::cout<<C(0,0)<<"\t"<<C(0,1)<<"\n";
    std::cout<<C(1,0)<<"\t"<<C(1,1)<<"\n";
   }

 void ModelAwas::Print21 (const Cost_du& C,const std::string S) const
  { 
    std::cout<<"\n"<<S<<"\n";
    std::cout<<C(0)<<"\n";
    std::cout<<C(1)<<"\n";
   }

 void ModelAwas::Print41 (const Cost_dx& C,const std::string S) const
  { 
    std::cout<<"\n"<<S<<"\n";
    std::cout<<C(0)<<"\n";
    std::cout<<C(1)<<"\n";
    std::cout<<C(2)<<"\n";
    std::cout<<C(3)<<"\n";
   }

  void ModelAwas::Print24 (const Cost_dxx  C,const std::string S) const {
   std::cout<<"\n"<<S<<"\n";
    std::cout<<C(0,0)<<"\t"<<C(0,1)<<"\t"<<C(0,2)<<"\t"<<C(0,3)<<"\n";
    std::cout<<C(1,0)<<"\t"<<C(1,1)<<"\t"<<C(1,2)<<"\t"<<C(1,3)<<"\n";
}
  void ModelAwas::Print42 (const Cost_dxx  C,const std::string S) const {
   std::cout<<"\n"<<S<<"\n";
   std::cout<<C(0,0)<<"\t"<<C(0,1)<<"\n";
   std::cout<<C(1,0)<<"\t"<<C(1,1)<<"\n";
   std::cout<<C(2,0)<<"\t"<<C(2,1)<<"\n";
   std::cout<<C(3,0)<<"\t"<<C(3,1)<<"\n";
}

  // Initialization without Stiffness (state : Theta and dTeheta/dT)
  ModelAwas::State_t ModelAwas::
  evolutionInit     (const State_t& state, const Control_t& control,const double r) const
  {
    State_t state_new(2);
    state_new(0) = state(0)+dT*state(1);
    state_new(1) = state(1)+dT*(-stiffness/inertia*r*r*state(0)-control(0));
    return state_new;
  }

  ModelAwas::State_dx ModelAwas:: 
  evolution_dxInit   (const double r) const
  {
    State_dx state_dx(2,2);
    state_dx(0,0) = 1;
    state_dx(0,1) = dT;
    state_dx(1,0) = -dT*stiffness/inertia*r*r;
    state_dx(1,1) = 1;
    return state_dx;
  }

ModelAwas::State_du ModelAwas::
  evolution_duInit   () const
  {
    State_du state_du(2,1);
    state_du(1,0) = -dT;
    state_du(0,0) = 0;
    return state_du;
  }


   ModelAwas::Cost_dx ModelAwas::
   integralCost_dxInit (const State_t& state,const double r) const
  {
    Cost_dx cost_dx(2);
    cost_dx(0) = Wx*2*stiffness*r*r*(stiffness*r*r*state(0)-Tdes);
    cost_dx(1) = 0.0;
    return cost_dx;
  }

  ModelAwas::Cost_du ModelAwas::
  integralCost_duInit (const Control_t& control) const
  {
    Cost_du cost_du(1);
    cost_du(0) = 2*Wu0*control(0);
    return cost_du;
  }

  ModelAwas::Cost_dxx ModelAwas::
  integralCost_dxxInit (const State_t& state,const double r) const
  {
    Cost_dxx cost_dxx(2,2);
    cost_dxx(0,0) = Wx*2*stiffness*stiffness*r*r*r*r;
    cost_dxx(0,1) =0;
    cost_dxx(1,0) =0;
    cost_dxx(1,1) =0;
    return cost_dxx;
  }

 ModelAwas::Cost_duu ModelAwas::
  integralCost_duuInit () const
  {
    Cost_duu cost_duu(1,1);
    cost_duu(0,0) = 2*Wu0;
    return cost_duu;
  }



} // namespace viaopt
