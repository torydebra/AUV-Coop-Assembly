#ifndef JACOBIANHELPER_H
#define JACOBIANHELPER_H

#include <Eigen/Core>
#include "infos.h"
#include "../../support/header/defines.h"
#include "../../support/header/formulas.h"
#include "../../support/header/conversions.h"

int computeWholeJacobianEE(struct Infos *robInfo);
int computeWholeJacobianTool(struct Infos *robInfo);


//int computeWholeJacobianTool_old(struct Infos *robInfo);
//int computeJacobianToolNoKdl(struct Infos *robInfo, std::string robotName);


#endif // JACOBIANHELPER_H
