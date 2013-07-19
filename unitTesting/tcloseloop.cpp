/* -------------------------------------------------------------------------- *
 * 
 * Unittest of the Model classes.
 * 
 * -------------------------------------------------------------------------- */


#include "viaopt/model-awas.hpp"
#include "viaopt/ilqr.hpp"
#include "viaopt/integrator-rk4.hpp"
#include <iostream>


using namespace viaopt;
int main (int , char** )
{
  typedef ModelAwas::State_t State_t;
  typedef ModelAwas::Control_t Control_t;
  typedef Ilqr<ModelAwas>::ControlList_t ControlList_t;
  typedef Ilqr<ModelAwas>::StateList_t StateList_t;

  /* --- MODEL---------*/
  ModelAwas model;

  /* --- ROBOT -------------------------------------------------------------- */
  IntegratorRK4<ModelAwas> robot;
  State_t state_t = model.initState(0,0.1); 
  robot.setState( state_t );

  /* --- CONTROL ------------------------------------------------------------ */
  Ilqr<ModelAwas> ilqr;
  ilqr.nbPreviewSteps = 15;
  ilqr.init();
  Ilqr<ModelAwas>::StateList_t Trajectory;
  Trajectory.push_back(state_t);
  Ilqr<ModelAwas>::ControlList_t ControlReal;

  /* --- MAIN LOOP ---------------------------------------------------------- */
  
  std::cout << "debut\n";
 State_t S = robot.getState();

   for (int cycle=0; cycle<0; ++cycle)
    {
      ilqr.computeControl (robot.getState());
      ControlList_t::iterator iter = ilqr.controlList.end();
      iter--;
      StateList_t::iterator iterState = ilqr.stateList.end(); iterState--;

       for (int i=0;i<model.window;++i)
	{
	  Control_t & control = *iter;
	  iter --;
	  // S = robot.integrate (control); // Nouvel etat -- Pb avec iRK4...
	  S = model.evolution(S,control); //Solution Temporaire !! pas état réel
	  std::cout<< "\nState : "<<S(0)<<" "<<S(1)<<" "<<S(2)<<" "<<S(3)<<"\n";
	  Trajectory.push_back(S); // Stockage de la trajectoire
	  ControlReal.push_back(S); // Stockage du controle

	  double couple = S(2)*S(2)*S(0)*model.stiffness/model.inertia; // couple instantané
       std::cout<<"COUPLE : "<<couple<<"\n";
 }
       robot.setState(S); // nouvel etat du robot
       std::cout<< "\n \n \n"; 
    }
}
