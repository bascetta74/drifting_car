#include <iostream>
#include <Eigen/Dense>
#include <matio.h>

void extractMatrix(Eigen::Ref<Eigen::MatrixXd> matrix, matvar_t **matrixMat, const int mat_idx);


int main(int argc, char **argv)
{
    // Open mat file
    const char *fileName = "/home/bascetta/Subversion/drifting_car/ROS/ags_controller/config/AGS_X0_HinfH2g_controller.mat";
    mat_t *mat = Mat_Open(fileName,MAT_ACC_RDONLY);

    if(mat)
    {
        std::cout << "mat file opened successfully!" << std::endl;

        // Reading struct controller
        // controller = struct with fields:
        //        Aki: cell array of matrix
        //        Bki: cell array of matrix
        //        Cki: cell array of matrix
        //        Dki: cell array of matrix
        //         Yi: cell array of matrix
        //         Xi: cell array of matrix
        //         Ai: cell array of matrix
        //         Bi: cell array of matrix
        //         Ci: cell array of matrix
        //         Di: cell array of matrix

        matvar_t *matVar = NULL;
        matVar = Mat_VarRead(mat, (char*)"controller") ;
        if(matVar)
        {
            // Get a pointer to the structure fields
            matvar_t **fields = (matvar_t **)matVar->data;
            matvar_t *Aki = (matvar_t *)fields[0];
            matvar_t *Bki = (matvar_t *)fields[1];
            matvar_t *Cki = (matvar_t *)fields[2];
            matvar_t *Dki = (matvar_t *)fields[3];
            matvar_t *Yi = (matvar_t *)fields[4];
            matvar_t *Xi = (matvar_t *)fields[5];
            matvar_t *Ai = (matvar_t *)fields[6];
            matvar_t *Bi = (matvar_t *)fields[7];
            matvar_t *Ci = (matvar_t *)fields[8];
            matvar_t *Di = (matvar_t *)fields[9];

            // Get number of elements in each field array
            int Aki_num = (int) Aki->dims[1];
            int Bki_num = (int) Bki->dims[1];
            int Cki_num = (int) Cki->dims[1];
            int Dki_num = (int) Dki->dims[1];
            int Yi_num = (int) Yi->dims[1];
            int Xi_num = (int) Xi->dims[1];
            int Ai_num = (int) Ai->dims[1];
            int Bi_num = (int) Bi->dims[1];
            int Ci_num = (int) Ci->dims[1];
            int Di_num = (int) Di->dims[1];

            // Get a pointer to field cell arrays
            matvar_t **Aki_matrix = (matvar_t **)Aki->data;
            matvar_t **Bki_matrix = (matvar_t **)Bki->data;
            matvar_t **Cki_matrix = (matvar_t **)Cki->data;
            matvar_t **Dki_matrix = (matvar_t **)Dki->data;
            matvar_t **Yi_matrix = (matvar_t **)Yi->data;
            matvar_t **Xi_matrix = (matvar_t **)Xi->data;
            matvar_t **Ai_matrix = (matvar_t **)Ai->data;
            matvar_t **Bi_matrix = (matvar_t **)Bi->data;
            matvar_t **Ci_matrix = (matvar_t **)Ci->data;
            matvar_t **Di_matrix = (matvar_t **)Di->data;

            // Create data structures to store matrices
            Eigen::MatrixXd Aki_eigenMat(Aki_matrix[0]->dims[0],Aki_matrix[0]->dims[1]);
            Eigen::MatrixXd Bki_eigenMat(Bki_matrix[0]->dims[0],Bki_matrix[0]->dims[1]);
            Eigen::MatrixXd Cki_eigenMat(Cki_matrix[0]->dims[0],Cki_matrix[0]->dims[1]);
            Eigen::MatrixXd Dki_eigenMat(Dki_matrix[0]->dims[0],Dki_matrix[0]->dims[1]);
            Eigen::MatrixXd Yi_eigenMat(Yi_matrix[0]->dims[0],Yi_matrix[0]->dims[1]);
            Eigen::MatrixXd Xi_eigenMat(Xi_matrix[0]->dims[0],Xi_matrix[0]->dims[1]);
            Eigen::MatrixXd Ai_eigenMat(Ai_matrix[0]->dims[0],Ai_matrix[0]->dims[1]);
            Eigen::MatrixXd Bi_eigenMat(Bi_matrix[0]->dims[0],Bi_matrix[0]->dims[1]);
            Eigen::MatrixXd Ci_eigenMat(Ci_matrix[0]->dims[0],Ci_matrix[0]->dims[1]);
            Eigen::MatrixXd Di_eigenMat(Di_matrix[0]->dims[0],Di_matrix[0]->dims[1]);

            // Extract matrices
            for (auto k=0; k<Aki_num; k++)
            {
                extractMatrix(Aki_eigenMat, Aki_matrix, k);
                std::cout << "Ak " << k+1 << ": " << std::endl << Aki_eigenMat << std::endl << std::endl;
            }
            for (auto k=0; k<Bki_num; k++)
            {
                extractMatrix(Bki_eigenMat, Bki_matrix, k);
                std::cout << "Bk " << k+1 << ": " << std::endl << Bki_eigenMat << std::endl << std::endl;
            }
            for (auto k=0; k<Cki_num; k++)
            {
                extractMatrix(Cki_eigenMat, Cki_matrix, k);
                std::cout << "Ck " << k+1 << ": " << std::endl << Cki_eigenMat << std::endl << std::endl;
            }
            for (auto k=0; k<Dki_num; k++)
            {
                extractMatrix(Dki_eigenMat, Dki_matrix, k);
                std::cout << "Dk " << k+1 << ": " << std::endl << Dki_eigenMat << std::endl << std::endl;
            }
            for (auto k=0; k<Yi_num; k++)
            {
                extractMatrix(Yi_eigenMat, Yi_matrix, k);
                std::cout << "Y " << k+1 << ": " << std::endl << Yi_eigenMat << std::endl << std::endl;
            }
            for (auto k=0; k<Xi_num; k++)
            {
                extractMatrix(Xi_eigenMat, Xi_matrix, k);
                std::cout << "X " << k+1 << ": " << std::endl << Xi_eigenMat << std::endl << std::endl;
            }
            for (auto k=0; k<Ai_num; k++)
            {
                extractMatrix(Ai_eigenMat, Ai_matrix, k);
                std::cout << "A " << k+1 << ": " << std::endl << Ai_eigenMat << std::endl << std::endl;
            }
            for (auto k=0; k<Bi_num; k++)
            {
                extractMatrix(Bi_eigenMat, Bi_matrix, k);
                std::cout << "B " << k+1 << ": " << std::endl << Bi_eigenMat << std::endl << std::endl;
            }
            for (auto k=0; k<Ci_num; k++)
            {
                extractMatrix(Ci_eigenMat, Ci_matrix, k);
                std::cout << "C " << k+1 << ": " << std::endl << Ci_eigenMat << std::endl << std::endl;
            }
            for (auto k=0; k<Di_num; k++)
            {
                extractMatrix(Di_eigenMat, Di_matrix, k);
                std::cout << "D " << k+1 << ": " << std::endl << Di_eigenMat << std::endl << std::endl;
            }
        }
        else
        {
            std::cout << "Cannot read variable from mat file!" << std::endl;
        }

        Mat_Close(mat);
    }
    else
    {
        std::cout << "Error opening the mat file" << std::endl;
        return 1;
    }
    return 0;
}


void extractMatrix(Eigen::Ref<Eigen::MatrixXd> matrix, matvar_t **matrixMat, const int mat_idx)
{
    // Get matrix dimensions
    int mat_row = (int) matrixMat[mat_idx]->dims[0];
    int mat_col = (int) matrixMat[mat_idx]->dims[1];

    // Initialize matrix
    matrix = Eigen::MatrixXd::Zero(mat_row, mat_col);

    // Get a pointer to matrix data
    double *matrixMat_data = (double *)matrixMat[mat_idx]->data;

    // Copy data to an eigen matrix
    int data_idx = 0;
    for (auto row=0; row<mat_row; row++)
    {
        for (auto col=0; col<mat_col; col++)
        {
            matrix(row,col) = matrixMat_data[data_idx++];
        }
    }
}