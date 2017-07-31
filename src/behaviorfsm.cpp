#include "debug.hpp"

#include "fsmlist.hpp"
#include "tinyfsm/tinyfsm.hpp"
#include <iostream>
#include "behaviorfsm.hpp"
using namespace std;

class KL; // forward declaration
class PLCL; // forward declaration
class PLCR; // forward declaration


// ----------------------------------------------------------------------------
// Transition functions
//

/*static void EmergencyCall() {
  cout << "*** calling Ambulance and Police ***" << endl;
}*/


// ----------------------------------------------------------------------------
// State: Ready
//

class Ready
: public BehaviorFSM
{
  void entry() override {
    //Do something
  }

  void react(CS const & ){
    cout << "Constant speed " << endl;
    transit<KL>();
  }
};


// ----------------------------------------------------------------------------
// State: Keep Line
//

class KL
: public BehaviorFSM
{
  void entry() override {
    cout<<"Entry KL"<<endl;
  }

  void react(MinCost const & e) override {
    cout << "Min cost: " << e.name << endl;

    if(e.name.compare("PLCL") == 0)
    {
      transit<PLCL>();
    } else if(e.name.compare("PLCR") == 0)
    {
      transit<PLCR>();
    }

  };
};


// ----------------------------------------------------------------------------
// State: Lane Change Left
//

class LCL
: public BehaviorFSM
{
  void entry() override {
    cout<<"Entry LCL"<<endl;
  }

  void react(MinCost const & e) override {
    cout << "Min cost: " << e.name << endl;
    if(e.name.compare("KL") == 0)
    {
      transit<KL>();
    }
  };
};

// ----------------------------------------------------------------------------
// State: Lane Change Right
//

class LCR
: public BehaviorFSM
{
  void entry() override {
    cout<<"Entry LCR"<<endl;
  }

  void react(MinCost const & e) override {
    cout << "Min cost: " << e.name << endl;
    if(e.name.compare("KL") == 0)
    {
      transit<KL>();
    }
  };
};

// ----------------------------------------------------------------------------
// State: Prepare Lane Change Left
//

class PLCL
: public BehaviorFSM
{
  void entry() override {
    cout<<"Entry PLCL"<<endl;
  }

  void react(MinCost const & e) override {
    cout << "Min cost: " << e.name << endl;
    if(e.name.compare("KL") == 0)
    {
      transit<KL>();
    }else if(e.name.compare("LCL") == 0)
    {
      transit<LCL>();
    }
  };
};

// ----------------------------------------------------------------------------
// State: Prepare Lane Change Right
//

class PLCR
: public BehaviorFSM
{
  void entry() override {
    cout<<"Entry PLCR"<<endl;
  }

  void react(MinCost const & e) override {
    cout << "Min cost: " << e.name << endl;
    if(e.name.compare("KL") == 0)
    {
      transit<KL>();
    }else if(e.name.compare("LCR") == 0)
    {
      transit<LCR>();
    }
  };
};

// ----------------------------------------------------------------------------
// Base state: default implementations
//

void BehaviorFSM::react(CS const &) {
  cout << "Constant speed ignored" << endl;
}

void BehaviorFSM::react(MinCost const &e) {
  cout << "MinCost ignored" << endl;
}


// ----------------------------------------------------------------------------
// Initial state definition
//
FSM_INITIAL_STATE(BehaviorFSM, Ready);
