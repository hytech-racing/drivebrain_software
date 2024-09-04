#ifndef __STATEESTIMATOR_H__
#define __STATEESTIMATOR_H__


// TODO:
// - [ ] implement the CAN driver connection that can create the flat state of the car 

// implement a thing that can maintain a "flexible" state of the car such that 
// it can build up a state of the car from the messages coming in. It starts out with an empty 
// state schema from the protobuf messages. all messages are assumed flat.

// - if a new message containing state information comes it (that has not arrived before), the state is "appended" to with the new value
// - if a new message containing state information comes in that has been seen before, the previous state is over-written.
//   each state update will have with it a timestamp



#endif // __STATEESTIMATOR_H__