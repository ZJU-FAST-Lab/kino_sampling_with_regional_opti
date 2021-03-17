#ifndef _QUADRATIC_FORM_H_
#define _QUADRATIC_FORM_H_

#include <Eigen/Eigen>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class QuadraticForm
{
public:
  QuadraticForm(){};
  
  QuadraticForm(int n)
  {

  };

  MatrixXd A;
  VectorXd a, c;

private:

};

#endif