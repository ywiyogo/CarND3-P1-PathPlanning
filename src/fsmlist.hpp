#ifndef FSMLIST_HPP_INCLUDED
#define FSMLIST_HPP_INCLUDED

#include "debug.hpp"
#include "tinyfsm/tinyfsm.hpp"

#include "behaviorfsm.hpp"
//#include "motor.hpp"

typedef tinyfsm::FsmList<BehaviorFSM> fsm_list;


/* wrapper to fsm_list::dispatch() */
template<typename E>
void send_event(E const & event)
{
  fsm_list::template dispatch<E>(event);
}


#endif
