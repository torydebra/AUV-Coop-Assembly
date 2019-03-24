#ifndef DEFINES_H
#define DEFINES_H

#define VEHICLE_DOF 6 // assuming vehicle full attuated
#define ARM_DOF 4 //CISP arm 4dof
#define TOT_DOF 10

///value for pseudoinverses
//threshold the threshold value for the SVD regularization; if the singular value
//is above this threshold, no regularization is performed on that singular value.
// default one, another can be choosen when construct the task object
#define THRESHOLD_DEFAULT 0.0001
//lambda the lambda value determines how "strong" is the regularization;
//it should be chosen accordingly with the threshold value
//(usually one order of magnitude smaller)
// default one, another can be choosen when construct the task object
#define LAMBDA_DEFAULT 0.00001

#define EQUALITY 0
#define INEQUALITY 1

#endif // DEFINES_H
