#ifndef LASTTASK_H
#define LASTTASK_H

#include "task.h" // task.h will include other header (like cmat and eigen)

/** @brief "Fake" Task needed as last to algorithm, it has all fixed matrices of eye and zeros so no setxx are necessary
 * updateMatrices is only necessary because father class Task say to implement it (it will be empty anyway)
*/
class LastTask : public Task {

public:
  LastTask(int dimension, int dof, bool eqType);
private: //to not use, so putted private
  int updateMatrices(struct Infos* const robInfo);
};


#endif // LASTTASK_H
