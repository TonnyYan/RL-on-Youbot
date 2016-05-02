#pragma once

typedef std::array<double, 3> Rayon; //vecteur à 3 dim r
typedef std::array<double, 5> Thetas;//vecteur à 5 dim Theta (seul les 4 premiers sont utilisés)

class Etat{
  Rayon r;
  Thetas theta;
  Thetas dthetamin;

public :

  Etat(const Rayon& r, const Thetas& theta, const Thetas& dthetamin)
  : r(r), theta(theta), dthetamin(dthetamin)  {
  }
  Etat()                       = default;
  Etat(const Etat&)            = default;
  Etat& operator=(const Etat&) = default;
  
  Rayon getRayon() const { return r;}
  Thetas getThetas() const { return theta;}
  Thetas getdThetas() const { return dthetamin;}

};

typedef std::vector<Etat> BaseEtats;

// We need to define a loss for Point comparison. It has to fit gaml::concept::Loss.



struct dThetas { //Sortie = 5 dThetas
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

class Loss {
public:
  typedef dThetas output_type;
  double operator()(const output_type& l1, const output_type& l2) const {
    gaml::loss::Quadratic<double> loss;
    return loss(l1.theta1, l2.theta1) + loss(l1.theta2, l2.theta2) + loss(l1.theta3, l2.theta3)+ loss(l1.theta4, l2.theta4);
  }
};
typedef std::pair<Entree,dThetas>  Data;
typedef std::vector<Data>         DataSet;
typedef  gaml::multidim::Predictor<dThetas,gaml::libsvm::Predictor<Entree,double>,5> fonction;	


// This function converts an array of 5 values to a Theta.
inline dThetas output_of_array(const std::array<double,5>& values) {
  dThetas res;
  res.theta1 = values[0];
  res.theta2 = values[1];
  res.theta3 = values[2];
  res.theta4 = values[3];
  res.theta5 = values[4];
  return res;
}
// This function extracts the scalar values from the Point.
inline std::array<double,5> array_of_output(const dThetas& output) {
  return {{output.theta1,output.theta2,output.theta3,output.theta4,output.theta5}};
}


inline int nb_nodes_of(const Entree& theta) {
  return 9;
}

inline void fill_nodes(const Entree& ent,struct svm_node* nodes) {
  
  nodes->index = 1;
  nodes->value = ent.x; 
  ++nodes;
  nodes->index = 2;
  nodes->value = ent.y;
  ++nodes;
  nodes->index = 3;
  nodes->value = ent.z; 
  ++nodes;
  nodes->index = 4;
  nodes->value = ent.theta1; 
  ++nodes;
  nodes->index = 5;
  nodes->value = ent.theta2; 
  ++nodes;
  nodes->index = 6;
  nodes->value = ent.theta3;
  ++nodes;
  nodes->index = 7;
  nodes->value = ent.theta4;
  ++nodes;
  nodes->index = 8;
  nodes->value = ent.theta5; 
  ++nodes;
  nodes->index = -1;   // end
}


inline Entree input_of (const Data& data) {return data.first;}
inline dThetas  output_of(const Data& data) {return data.second;}

inline fonction calcul_f(const BaseEtats& baseEtats);




