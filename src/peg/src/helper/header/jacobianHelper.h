#ifndef JACOBIANHELPER_H
#define JACOBIANHELPER_H

#include <Eigen/Core>
#include "infos.h"
#include "../../support/header/defines.h"
#include "../../support/header/formulas.h"

int computeWholeJacobianEE(struct Infos *robInfo);
int computeWholeJacobianTool(struct Infos *robInfo);


#endif // JACOBIANHELPER_H
