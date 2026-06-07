#include "cubesat_captain/expert.hpp"
namespace cubesat_captain {
    class UnfoldingExpert: public Expert{
        void enter_state() override;
    };
    class AutoCameraExpert : public Expert{
        void enter_state() override;
    };
}