#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <urdf/model.h>

#include "support/header/conversions.h"


int main(int argc, char **argv){


  std::string filename = "/home/tori/UWsim/UWsim/src/uwsim/data/scenes/g500ARM5.urdf";
  TiXmlDocument file_xml(filename);
  file_xml.LoadFile();

  // parse URDF arm model
  KDL::Tree my_tree;
  urdf::Model my_model;
  if (!my_model.initXml(&file_xml)){
     ROS_ERROR("Failed to parse urdf robot model");
     return false;
  }
  if (!kdl_parser::treeFromUrdfModel(my_model, my_tree)){
     ROS_ERROR("Failed to construct kdl tree");
     return false;
  }
  KDL::Chain myChain;
  my_tree.getChain("base_link", "part4_base", myChain);
  //std::cout<< myChain.getNrOfJoints() << "\n";
  KDL::ChainJntToJacSolver solver(myChain);

  KDL::JntArray jointpositions = KDL::JntArray(myChain.getNrOfJoints());

  std::vector<double> pos;
  pos.push_back(0.1);
  pos.push_back(0.5);
  pos.push_back(0.8);
  pos.push_back(1.0);
  pos.push_back(0.15);
  for(unsigned int i=0; i<myChain.getNrOfJoints(); i++){

         jointpositions(i)=pos[i];
  }

  KDL::Jacobian jacob;
  jacob.resize(myChain.getNrOfJoints());
  solver.JntToJac(jointpositions, jacob);


  Eigen::MatrixXd jacob_eig = CONV::jacobian_kdl2eigen(jacob);
  std::cout << jacob.data << "\n\n";
  std::cout << jacob_eig << "\n\n";


}

