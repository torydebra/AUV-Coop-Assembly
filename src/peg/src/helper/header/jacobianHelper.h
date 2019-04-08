#ifndef JACOBIANHELPER_H
#define JACOBIANHELPER_H

#include <Eigen/Core>
#include "infos.h"
#include "../../support/header/defines.h"
#include "../../support/header/formulas.h"

int computeWholeJacobian(struct Infos *robInfo, ControlPoint pt);


#endif // JACOBIANHELPER_H
