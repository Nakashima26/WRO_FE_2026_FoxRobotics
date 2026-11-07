#include <float.h>
#include "fox_native.h"
FOX_API int32_t fox_abi_version(void){return 1;}
FOX_API int32_t fox_flt_eval_method(void){return (int32_t)FLT_EVAL_METHOD;}
FOX_API double fox_probe_contract_d(double a,double b,double c){return a*b+c;}
FOX_API float fox_probe_contract_f(float a,float b,float c){return a*b+c;}
