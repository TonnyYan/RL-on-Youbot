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
#include "learning_follow.hpp"

 
struct svm_parameter params;

void Initialisation(){
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


fonction calcul_f(const BaseEtats& baseEtats, bool record){

  std::list<gaml::libsvm::Predictor<Entree,double>> null_predictors;
  gaml::libsvm::Predictor<Entree,double> null_predictor(nb_nodes_of, fill_nodes);
  for(int i = 0; i < 4; i++) null_predictors.push_back(null_predictor);
  fonction f(output_of_array,null_predictors.begin(), null_predictors.end()); // Dummy init, will be ecrased by learner.

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
	baseEtats[i].getThetas()[3]
      };
      dThetas s = {
	baseEtats[i].getdThetas()[0],
	baseEtats[i].getdThetas()[1], 
	baseEtats[i].getdThetas()[2],
	baseEtats[i].getdThetas()[3]
      };
      data = Data(e,s);
      i++;
    }


    //Learner : prend une base de données, pond un algo
    //f : prend un algo, sort des prédictions

    auto scalar_learner = gaml::libsvm::supervized::learner<Entree,double>(params, nb_nodes_of, fill_nodes); 

    auto learner = gaml::multidim::learner<dThetas,4>(scalar_learner, array_of_output, output_of_array);
    std::cout << "Learning..." << std::endl;
    f = learner(basis.begin(), basis.end(), input_of, output_of);




    if(record){//Enregistrement
      auto f_predictors                   = f.predictors();
      std::array<std::string,4> filenames = {{std::string("theta1.pred"),"theta2.pred","theta3.pred","theta4.pred"}};
      auto name_iter                      = filenames.begin();
      for(auto& pred : f_predictors) pred.save_model(*(name_iter++));
      std::cout << "Enregistrement" << std::endl;
    }



    // We can compute the empirical risk with gaml tools.
    auto evaluator = gaml::risk::empirical(Loss());
    double risk = evaluator(f, basis.begin(), basis.end(), input_of, output_of);
    std::cout << "Empirical risk : " << risk << std::endl
	      << "    sqrt(risk) : " << sqrt(risk) << std::endl;
  }

  catch(gaml::exception::Any& e) {
    std::cout << e.what() << std::endl;
  }

  return f;
}
