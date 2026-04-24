#include "fairino_hardware/global_val_def.hpp"

double* global_exaxis_pos(){
    static double tmp[4]{0.};
    return tmp;
}