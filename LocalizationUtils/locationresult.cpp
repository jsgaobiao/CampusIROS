#include "locationresult.h"

LocationResult::LocationResult()
{
    for(int i = 0; i != 3; i++){
        this->bestParticleBelief[i] = 0.0;
        this->resultEvaluation[i] = 0.0;
        this->featurePtsNum[i] = 0;
    }
    this->pfConverged = false;
}
