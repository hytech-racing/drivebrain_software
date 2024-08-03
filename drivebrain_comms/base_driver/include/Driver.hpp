#pragma once


// TODO
// - [ ] 
namespace comms
{
/// @brief This class is what all drivers implement / look like. 
//         They will all have a parameter update function that has to be registered, 
//         a function for initialization, handlers for rx and tx (will be ran in threads), handl

class Driver 
{
    public: 
        Driver() = default;
        ~Driver() = default;

        virtual bool init() = 0;
        virtual bool init() = 0;

    private:
         

};
}