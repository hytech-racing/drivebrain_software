
## first pass
ideas:
- at first, we will generate protobuf message descriptions from the simulink model
- we will do handle manual integration for any new inputs / outputs

questions:
1. do we want to separate the state estimation and the controllers?
    - I think so since we can iterate on controllers faster than state estimation and switch between them using the same plugin/library defined state estimation systems.

2. does it really make sense to have the state estimation be separated from the controller when they are both developed in simulink?
    - maybe...?
    - how will we handle having both the state estimation and the controller in simulink lap sim AND having to have wrapper code for when it becomes a separate lib integration?

I think for q2, we can go with keeping the state estimation monolithic? 

- keep CASE, just new wrapper code for v1? 

## prototyping
idea: if we have the ability to go both ways for having the ability to both:
 
1. define/add inputs for a simulink model based on a .proto message

2. generate a .proto message from either inputs OR outputs of a simulink model

and generated code for the data passing into and out of proto messages, then we can book-end the controllers and estimators. 

## TODO

- [ ] make a controller manager for switching between controllers
- [ ] make an interface for the foxglove parameter server

    - how will we hook this up into the controller and state estimator?