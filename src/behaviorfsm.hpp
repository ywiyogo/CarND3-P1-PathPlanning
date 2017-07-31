#ifndef BEHAVIORFSM_HPP_INCLUDED
#define BEHAVIORFSM_HPP_INCLUDED

#include "tinyfsm/tinyfsm.hpp"
#include <string>
using namespace std;

// ----------------------------------------------------------------------------
// Event declarations
//
struct MinCost : tinyfsm::Event
{
  string name;
};
struct CS       : tinyfsm::Event { };



// ----------------------------------------------------------------------------
// Car (FSM base class) declaration
//

class BehaviorFSM
: public tinyfsm::Fsm<BehaviorFSM>
{
  /* NOTE: react(), entry() and exit() functions need to be accessible
   * from tinyfsm::Fsm class. You might as well declare friendship to
   * tinyfsm::Fsm, and make these functions private:
   *
   * friend class Fsm;
   */
public:

  /* default reaction for unhandled events */
  void react(tinyfsm::Event const &) { };
  virtual void react(CS const &);
  virtual void react(MinCost const &);

  virtual void entry(void) { };  /* entry actions in some states */
  void         exit(void)  { };  /* no exit actions at all */

protected:

  int v;
  int a;
  int s;
  int d;
};


#endif
