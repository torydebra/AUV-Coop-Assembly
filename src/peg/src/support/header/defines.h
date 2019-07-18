#ifndef DEFINES_H
#define DEFINES_H

#define PI_TORI 3.141592654

#define VEHICLE_DOF 6 // these are the DOF considering the 3d space movement, are not
                      // the real vehicle dof
#define ARM_DOF 4 //CSIP ARM5E has actually 4dof (the 5th is open/close gripper)
#define TOT_DOF 10

///value for pseudoinverses
//threshold the threshold value for the SVD regularization; if the singular value
//is above this threshold, no regularization is performed on that singular value.
// default one, another can be choosen when construct the task object
#define THRESHOLD_DEFAULT 0.01 //0.01 original
//lambda the lambda value determines how "strong" is the regularization;
//it should be chosen accordingly with the threshold value
//(usually one order of magnitude smaller)
// default one, another can be choosen when construct the task object
#define LAMBDA_DEFAULT 0.0001  //0.0001 original

///Joint limits (in rad) (slew shoulder elbow jawrotate)
/// find in the urdf file of the robot
#define JLIM1_MIN -PI_TORI/2
#define JLIM1_MAX 0.5488
#define JLIM2_MIN 0
#define JLIM2_MAX 1.58665
#define JLIM3_MIN 0
#define JLIM3_MAX 2.15294
//not in urdf file, probably they are default value
//find this one with keyboard command, the jaw stops rotating at these values
#define JLIM4_MIN -PI_TORI
#define JLIM4_MAX PI_TORI

//longest dimension of the vehicle (for avoid crash between two robot)
#define VEH_DIM 1.5 //meters, according to datasheet of real robo, check if in the simul is the same

// usato per ee avoid obstacle (avoid seafloor)
//TODO usare il laser di profondità invece?
#define SEAFLOOR_DEPTH 9 //i.e. "object" terrain position plus high (considering z axis of world pointing down

// decide the control point (end effector or tool) that is the one which will reach the goal
enum ControlPoint {ee, tool};

//log
#define LOG 1 //0-1 logging (printing matrix on files) not activated or activated


#define MS_CONTROL_LOOP 100 //millisecond for the control loops

#define COLLISION_PROPAGATOR true //to set if collision with peg and hole cause disturbances to arm
#define GRASP_CONSTRAINER true //to firm grasp constrain, which bring ee towards the correspondent point of the other robot peg
#define CHANGE_GOAL true //to change the goal accordingly to the force arrives
#endif // DEFINES_H
