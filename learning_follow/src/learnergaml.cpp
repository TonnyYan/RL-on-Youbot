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

// Let us redefine the oracle with our types
// #define NOISE_LEVEL .1
// Point oracle(double theta) {
//   return {(.5-NOISE_LEVEL)*(1+cos(2*theta))            + gaml::random::uniform(-NOISE_LEVEL,NOISE_LEVEL),  // x(theta)
//           (.5-NOISE_LEVEL)*(1+sin(2*theta))            + gaml::random::uniform(-NOISE_LEVEL,NOISE_LEVEL),  // y(theta)
//           NOISE_LEVEL+(1-2*NOISE_LEVEL)*theta/(2*M_PI) + gaml::random::uniform(-NOISE_LEVEL,NOISE_LEVEL)}; // z(theta)
// }
// // We need a function that builds an array of svm_nodes for
// // representing some input. When the input is a collection of values
// // that can provide iterators, libsvm::input_of can help. Here, we
// // have to write it by ourselves. Input is only a scalar.



void calcul_f(BaseEtats baseEtats){

  // Let us make libsvm quiet
  gaml::libsvm::quiet();
  // random seed initialization
  std::srand(std::time(0));

  try {
    // Let us collect samples.
    DataSet basis;
    int i =0;
    basis.resize(baseEtats.size());
    for(auto& data : basis) {
     Entree e = {
       baseEtats[i].getRayon()[0],
	baseEtats[i].getRayon()[1],
       baseEtats[i].getRayon()[2],
       baseEtats[i].getThetas()[0],
	baseEtats[i].getThetas()[1], 
	baseEtats[i].getThetas()[2],
	baseEtats[i].getThetas()[3],
	baseEtats[i].getThetas()[4]
      };
      dThetas s = {
	baseEtats[i].getdThetas()[0],
	baseEtats[i].getdThetas()[1], 
	baseEtats[i].getdThetas()[2],
	baseEtats[i].getdThetas()[3],
	baseEtats[i].getdThetas()[4]
      };
      data = Data(e,s);
      i++;
    }


    // Let us set configure a svm   ????
    struct svm_parameter params;
    gaml::libsvm::init(params);
    params.kernel_type = RBF;          // RBF kernel
    params.gamma       = 1;            // k(u,v) = exp(-gamma*(u-v)^2)
    params.svm_type    = EPSILON_SVR;
    params.p           = .01;          // epsilon
    params.C           = 10;
    params.eps         = 1e-10;        // numerical tolerence

    // This sets up a svm learning algorithm for predicting a scalar.
    auto scalar_learner = gaml::libsvm::supervized::learner<Entree,double>(params, nb_nodes_of, fill_nodes); //Learner : prend une base de données, pond un algo

    // Let us use it for learning x, y and z.
    auto learner = gaml::multidim::learner<dThetas,5>(scalar_learner, array_of_output, output_of_array);

    // Let us train it and get some predictor f. f is a function, as the oracle.
    std::cout << "Learning..." << std::endl;
    auto f = learner(basis.begin(), basis.end(), input_of, output_of);//f : prend un algo, sort des prédictions

    // Let us retrieve the three SVMs in order to save each one.
    auto f_predictors                   = f.predictors();
    std::array<std::string,5> filenames = {{std::string("theta1.pred"),"theta2.pred","theta3.pred","theta4.pred","theta5.pred"}};
    auto name_iter                      = filenames.begin();
    for(auto& pred : f_predictors) pred.save_model(*(name_iter++));
    // We can compute the empirical risk with gaml tools.
    auto evaluator = gaml::risk::empirical(Loss());
    double risk = evaluator(f, basis.begin(), basis.end(), input_of, output_of);
    std::cout << "Empirical risk : " << risk << std::endl
              << "    sqrt(risk) : " << sqrt(risk) << std::endl;

  }
  catch(gaml::exception::Any& e) {
    std::cout << e.what() << std::endl;
  }
  
}
