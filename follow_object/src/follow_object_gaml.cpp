#include <gaml-libsvm.hpp>
#include <cmath> 
#include <cstdlib>
#include <vector>
#include <utility>
#include <string>
#include <fstream>
#include <iostream>
#include <ctime>
#include <array>
#include <list>
#include "Etats.hpp"
 
struct svm_parameter params;

void Initialisation_params(){

  gaml::libsvm::quiet();
  std::srand(std::time(0));
  gaml::libsvm::init(params);
  params.kernel_type = RBF;          // RBF kernel
  params.gamma       = 1;            // k(u,v) = exp(-gamma*(u-v)^2)
  params.svm_type    = EPSILON_SVR;
  params.p           = .01;          // epsilon
  params.C           = 10;
  params.eps         = 1e-10;        // numerical tolerence


}
