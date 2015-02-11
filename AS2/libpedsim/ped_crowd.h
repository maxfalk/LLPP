#ifndef _ped_model_h_
#define _ped_model_h_

namespace Ped{
    class Crowd {
        public:
            float *AgentsX;
            float *AgentsY;
            float *AgentsZ;

            float *MoveForceX;
            float *MoveForceY;
            float *MoveForceZ;

            void go();
            void where_to_go();
        private:

    }
}
#endif
