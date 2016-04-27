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


struct dThetas { //Sortie
  double theta1;
  double theta2;
  double theta3;
  double theta4;
  double theta5;
};

struct Entree { //Rayon+Thetas qui font une entree
  double x;
  double y;
  double z;
  double theta1;
  double theta2;
  double theta3;
  double theta4;
  double theta5;
};

// We need to define a loss for Point comparison. It has to fit gaml::concept::Loss.
class Loss {
public:
  typedef dThetas output_type;
  double operator()(const output_type& l1, const output_type& l2) const {
    gaml::loss::Quadratic<double> loss;
    return loss(l1.theta1, l2.theta1) + loss(l1.theta2, l2.theta2) + loss(l1.theta3, l2.theta3)+ loss(l1.theta4, l2.theta4);
  }
};
typedef std::pair<Entree,Thetas>  Data;
typedef std::vector<Data>         DataSet;


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
int nb_nodes_of(const double& theta) {
  return 9;
}
void fill_nodes(const Entree& ent,struct svm_node* nodes) {
  nodes[0].index = 1;
  nodes[0].value = ent.x; 
  nodes[1].index = 1;
  nodes[1].value = ent.y;
  nodes[2].index = 1;
  nodes[2].value = ent.z; 
  nodes[3].index = 1;
  nodes[3].value = ent.theta1; 
  nodes[4].index = 1;
  nodes[4].value = ent.theta2; 
  nodes[5].index = 1;
  nodes[5].value = ent.theta3;
  nodes[6].index = 1;
  nodes[6].value = ent.theta4;
  nodes[7].index = 1;
  nodes[7].value = ent.theta5; 
  nodes[1].index = -1;   // end
}
double input_of (const Data& data) {return data.first;}
Point  output_of(const Data& data) {return data.second;}
// This function converts an array of 5 values to a Theta.
dThetas output_of_array(const std::array<double,5>& values) {
  dThetas res;
  res.theta1 = values[0];
  res.theta2 = values[1];
  res.theta3 = values[2];
  res.theta4 = values[3];
  res.theta5 = values[4];
  return res;
}
// This function extracts the scalar values from the Point.
std::array<double,5> array_of_output(const dThetas& output) {
  return {{output.theta1,output.theta2,output.theta3,output.theta4,output.theta5}};
}
int main(int argc, char* argv[]) {


  // Let us make libsvm quiet
  gaml::libsvm::quiet();
  // random seed initialization
  std::srand(std::time(0));


  try {
    // Let us collect samples.
    DataSet basis;
    Etat baseEtats;
    int i =0;
    basis.resize(baseEtats.size());
    for(auto& data : basis) {
     Entree e = {
	baseEtats[i].r.x,
	baseEtats[i].r.y,
	baseEtats[i].r.z,
	baseEtats[i].theta.theta1,
	baseEtats[i].theta.theta2, 
	baseEtats[i].theta.theta3,
	baseEtats[i].theta.theta4,
	baseEtats[i].theta.theta5
      };
      dThetas s = {
	baseEtats[i].dthetamin.theta1,
	baseEtats[i].dthetamin.theta2,
	baseEtats[i].dthetamin.theta3,
	baseEtats[i].dthetamin.theta4,
	baseEtats[i].dthetamin.theta5
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
    auto scalar_learner = gaml::libsvm::supervized::learner<Entree,double>(params, nb_nodes_of, fill_nodes);
    // Let us use it for learning x, y and z.
    auto learner = gaml::multidim::learner<dThetas,5>(scalar_learner, array_of_output, output_of_array);
    // Let us train it and get some predictor f. f is a function, as the oracle.
    std::cout << "Learning..." << std::endl;
    auto f = learner(basis.begin(), basis.end(), input_of, output_of);
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
    // Let us load our predictor. We need first to gather each loaded
    // scalar predictor in a collection...
    std::list<gaml::libsvm::Predictor<Entree,double>> predictors;
    name_iter= filenames.begin();
    for(unsigned int dim = 0; dim < 5; ++dim) {
      gaml::libsvm::Predictor<Entree,double> predictor(nb_nodes_of, fill_nodes);
      predictor.load_model(*(name_iter++));
      predictors.push_back(predictor);
    }
    // ... and create the 3D predictor (variable g) from the
    // collection of scalar predictors.
    gaml::multidim::Predictor<dThetas,
                              gaml::libsvm::Predictor<Entree,double>,
                              5> g(output_of_array, predictors.begin(), predictors.end());
    // let us gnuplot what we have.
    std::ofstream dataset_file("dataset.data");
    for(auto& data : basis)
      dataset_file << data.first.x << ' ' << data.first.y << ' ' << data.first.z << ' ' << data.first.theta1 << ' ' << data.first.theta2 << ' ' << data.first.theta3 << ' ' << data.first.theta4 << ' '<< data.first.theta5 << ' ' << data.second.theta1 << ' ' << data.second.theta2 << ' ' << data.second.theta3 << ' ' << data.second.theta4 <<' ' << data.second.theta5 << std::endl;
    dataset_file.close();
    std::ofstream prediction_file("prediction.data");
    for(double theta = 0; theta <= 2*M_PI; theta += .01) {
      dThetas p = g(theta);
      prediction_file << theta << ' ' << p.theta1   << ' ' << p.theta2   << ' ' << p.theta3 << ' ' << p.theta4 << ' ' << p.theta5 << std::endl;
    }
    prediction_file.close();
    std::ofstream gnuplot_file_3d("3D.plot"); 
    gnuplot_file_3d << "set ticslevel 0" << std::endl
                    << "set xlabel 'x(theta)'" << std::endl
                    << "set ylabel 'y(theta)'" << std::endl
                    << "set zlabel 'z(theta)' " << std::endl
                    << "splot 'dataset.data' using 2:3:4 with points notitle, "
                    << "'prediction.data' using 2:3:4 with lines notitle" << std::endl;
    gnuplot_file_3d.close();
    std::cout << std::endl
              << "Try 'gnuplot -p 3D.plot'" << std::endl;
    std::ofstream gnuplot_file_1d("1D-x.plot"); 
    gnuplot_file_1d << "set xlabel 'theta'" << std::endl
                    << "set ylabel 'x(theta)'" << std::endl
                    << "plot 'dataset.data' using 1:2 with points notitle, "
                    << "'prediction.data' using 1:2 with lines notitle" << std::endl;
    gnuplot_file_1d.close();
    std::cout << "Try 'gnuplot -p 1D-x.plot'" << std::endl
              << std::endl;
  }
  catch(gaml::exception::Any& e) {
    std::cout << e.what() << std::endl;
  }
  
  return 0;
}
