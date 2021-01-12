#include "dvs_data_tools/utils.h"

dvsUtils::dvsUtils(){
}

dvsUtils::~dvsUtils(){}

/** This function was written by
 * @author  Raul Tapia (raultapia _at_ us.es | github.com/raultapia)
 */

bool dvsUtils::inEdge(unsigned int xx, unsigned int yy){
        double x = (double)xx;
        double y = (double)yy;
        return ( \
                       y<115 && x>179 && sqrt(pow(y-115,2) + pow(x-179,2)) > 182 || \
                       y>135 && x>245 && sqrt(pow(y-135,2) + pow(x-245,2)) > 131 || \
                       y>135 && x>260 && sqrt(pow(y-135,2) + pow(x-260,2)) > 124 || \
                       y<115 && x<160 && sqrt(pow(y-115,2) + pow(x-160,2)) > 165 || \
                       y>115 && x<160 && sqrt(pow(y-115,2) + pow(x-160,2)) > 178 || \
                       y>115 && x<105 && sqrt(pow(y-115,2) + pow(x-105,2)) > 144 \
                       );
}