#ifndef __MATLABMATH_H__
#define __MATLABMATH_H__

#include <utility>
#include <Configurable.hpp>
#include <VehicleDataTypes.hpp>
#include <hytech_msgs.pb.h> // for TireDynamics

#include <Tire_Model_Codegen.h> // code-gend

namespace estimation
{
    class MatlabMath : public core::common::Configurable
    {
        public:
            struct config
            {
                // live configs
                float lmux_fl;
                float lmuy_fl;
                float lmux_fr;
                float lmuy_fr;
                float lmux_rl;
                float lmuy_rl;
                float lmux_rr;
                float lmuy_rr;
                bool use_fake_data;
                float Fake_Vx;
                float DriveBiasFront;
                float BrakeBiasFront;
                // file loaded configs
                float x1_fl;
                float x2_fl;
                float x3_fl;
                float y1_fl;
                float y2_fl;
                float y3_fl;
                float x1_fr;
                float x2_fr;
                float x3_fr;
                float y1_fr;
                float y2_fr;
                float y3_fr;
                float x1_rl;
                float x2_rl;
                float x3_rl;
                float y1_rl;
                float y2_rl;
                float y3_rl;
                float x1_rr;
                float x2_rr;
                float x3_rr;
                float y1_rr;
                float y2_rr;
                float y3_rr;
            };
        
            MatlabMath(core::Logger &logger, core::JsonFileHandler &json_file_handler, bool &construction_failed);

            // TODO
            // hytech_msgs::TireDynamics convert_to_proto()

            std::pair<core::MatlabMathResult, core::ControllerTorqueOut> evaluate_estimator(const core::VehicleState &current_state, const core::RawInputData& raw_input);
            bool init();
            
        private:
            config _config;
            std::mutex _config_mutex;
            Tire_Model_Codegen::ExtU_Tire_Model_Codegen_T _inputs;
            Tire_Model_Codegen _model;
            void _handle_param_updates(const std::unordered_map<std::string, core::common::Configurable::ParamTypes> &new_param_map);

    }; 
}
#endif // __MATLABMATH_H__