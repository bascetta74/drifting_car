#ifndef CPLEXSOLVER_CWRAPPER_H_
#define CPLEXSOLVER_CWRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

int init_CPLEXsolver_cwrapper(const int numVariable, const int numIneqConstraint, const int numEqConstraint);
int deinit_CPLEXsolver_cwrapper();

int initProblem();
int setProblem(double* hessian_mat, double* gradient_vect);
int solveProblem(double* result_vect);
/*
bool setProblem(const Ref<const MatrixXd> hessian, const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B);
bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                const Ref<const VectorXd> gradient);
bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B);
bool setProblem(const std::vector<double>& lowerBound, const std::vector<double>& upperBound, const Ref<const MatrixXd> hessian,
                const Ref<const VectorXd> gradient, const Ref<const MatrixXd> A, const Ref<const VectorXd> B, const Ref<const MatrixXd> Aeq,
                const Ref<const VectorXd> Beq);
*/

#ifdef __cplusplus
}
#endif

#endif /* CPLEXSOLVER_CWRAPPER_H_ */