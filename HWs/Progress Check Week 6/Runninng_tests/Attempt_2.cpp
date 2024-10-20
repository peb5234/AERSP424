#include <iostream>
#include <cmath>
#include <vector>
using namespace std;


double ffunc(double t,double &p,double &q, double &r);
void derivative_function(vector<double> x, vector<vector<double>> dcm, vector<double> v_body, double time,vector<double> &xdot,vector<vector<double>> &Cdot);
double dotProduct(vector<vector<double>> a, vector<vector<double>> b);
vector<vector<double>> multiplyMatrices(vector<vector<double>>& mat1, vector<vector<double>>& mat2);
vector<double> addVectors(vector<double> a, vector<double> b);
vector<double> scaleVector(vector<double> vec, double scalar);
vector<vector<double>> scaleMatrix(vector<vector<double>> &vec, double scalar);
vector<vector<double>> addMatrix(vector<vector<double>> a, vector<vector<double>> b);
vector<double> add4Vectors( vector<double> a, vector<double> b, vector<double> c, vector<double> d);
vector<vector<double>> add4Matrix(vector<vector<double>> a, vector<vector<double>> b, vector<vector<double>> c, vector<vector<double>> d);
 


int main()
{
    const double PI=3.1414926;
    double dt; //time step
    cout<<"Put in the dt value:"<<endl;
    cin>>dt;
    vector<double> x0 = {0.0, 0.0, 0.0}; // radians
    vector<double> V_body = {60.0 * 6076.0 / 3600.0, 0.0, 0.0}; // Convert knots to ft/sec
    vector<vector<double>> x= {
        {x0[0], x0[1], x0[2]},
        {V_body[0], V_body[1], V_body[2]},
        {0.0, 0.0, 0.0}
    }; // 2D vector to store state over time
    //cout<<"0"<<endl;
    //x.push_back({x0[0], x0[1], x0[2], V_body[0], V_body[1], V_body[2], 0.0, 0.0, 0.0});
    vector<vector<double>> xd={}; //derivatives vector
    vector<double> t = {0.0}; //time vector
    double tmax = 60.0; // Simulation time limit (60 seconds)
    int func_calls = 0; // To track function calls
    double phi = x[0][0];
    double theta = x[0][1];
    double psi = x[0][2];
    //cout<<"1"<<endl;
    // Initialize DCM (Direction Cosine Matrix) as identity matrix (3x3)
    vector<vector<double>> identity_DCM = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    // phi DCM
    vector<vector<double>> DCM_phi = {
        {1.0, 0.0, 0.0},
        {0.0, cos(phi), -sin(phi)},
        {0.0, sin(phi), cos(phi)}
    };

    // theta DCM
    vector<vector<double>> DCM_theta = {
        {cos(theta), 0.0, sin(theta)},
        {0.0, 1, 0.0},
        {-sin(theta), 0, cos(theta)}
    };
    // psi DCM
    vector<vector<double>> DCM_psi = {
        {cos(psi), -sin(psi), 0.0},
        {sin(psi), cos(psi), 1},
        {0.0, 0.0, 1}
    };

    //Multiply the DCMs
    vector<vector<double>> intermediate_DCM;
    vector<vector<double>> DCM;
    intermediate_DCM=multiplyMatrices(identity_DCM,DCM_phi);
    //cout<<"2"<<endl;
    intermediate_DCM=multiplyMatrices(intermediate_DCM,DCM_theta);
    //cout<<"3"<<endl;
    DCM=multiplyMatrices(intermediate_DCM,DCM_psi);
    //cout<<"4"<<endl;

    double current_time;
    
    vector<double> xdot1;
    vector<vector<double>> Cdot1;
    vector<double> xdot2;
    vector<vector<double>> Cdot2;
    vector<double> xdot3;
    vector<vector<double>> Cdot3;
    vector<double> xdot4;
    vector<vector<double>> Cdot4;

    vector<double> xdot;
    vector<vector<double>> Cdot;

    for (int index=0;index<(int)(tmax/dt);index++){
        current_time= index*dt;
        vector<double> xn= x[index];
        //cout<<"5"<<endl;
        
        //Getting Rk4 Steps
        size_t rows = DCM.size();
        // Get the number of columns (assuming at least one row)
        size_t cols = rows > 0 ? DCM[0].size() : 0;
    
        // Output the size
        //cout << "Number of rows: " << rows << endl;
        //cout << "Number of columns: " << cols << endl;
        derivative_function(xn,DCM,V_body,current_time,xdot1,Cdot1);
        //cout<<"DEBUG6"<<endl;
    
    // Check the size of vector 
        //std::cout << "Number of values in vector xdot1: " <<   xdot1.size() << std::endl;
        derivative_function(addVectors(xn,scaleVector(xdot1,dt/2.0)),addMatrix(DCM,scaleMatrix(Cdot1,dt/2.0)),V_body ,current_time+dt/2,xdot2,Cdot2);
        //cout<<"DEBUG7"<<endl;
        derivative_function(addVectors(xn,scaleVector(xdot2,dt/2)),addMatrix(DCM,scaleMatrix(Cdot2,dt/2)),V_body ,current_time+dt/2,xdot3,Cdot3);
        derivative_function(addVectors(xn,scaleVector(xdot3,dt)),addMatrix(DCM,scaleMatrix(Cdot3,dt)),V_body ,current_time+dt,xdot4,Cdot4);
        
        //Getting Rk4 xdot cdot
        xdot=add4Vectors(xdot1,scaleVector(xdot2,2),scaleVector(xdot3,2),xdot4);
        //cout<<"DEBUG8"<<endl;
        xdot=scaleVector(xdot,1/6);
        Cdot=add4Matrix(Cdot1,scaleMatrix(Cdot2,2),scaleMatrix(Cdot3,2),Cdot4);
        //cout<<"DEBUG9"<<endl;
        Cdot=scaleMatrix(Cdot,1/6);
        //cout<<"DEBUG10"<<endl;
        
        DCM=addMatrix(DCM,scaleMatrix(Cdot,dt));
        //cout<<"DEBUG11"<<endl;
        x.push_back(addVectors(xn,scaleVector(xdot,dt)));
        //cout<<"DEBUG12"<<endl;
        xd.push_back(xdot);
        //cout<<"DEBUG13"<<endl;


        t.push_back(current_time+dt);
        func_calls+=4;

        cout << "t: " << current_time << ", ";
        cout << "phi_dot: " << xd[index][0] << ", ";
        cout << "theta_dot: " << xd[index][1] << ", ";
        cout << "psi_dot: " << xd[index][2] << ", ";
        cout << "phi: " << xn[0] << ", ";
        cout << "theta: " << xn[1] << ", ";
        cout << "psi: " << xn[2] << ", ";
        cout << "Vn: " << xdot[0] << ", "; // Adjust if needed
        cout << "Ve: " << xdot[1] << ", "; // Adjust if needed
        cout << "Vd: " << xdot[2] << ", "; // Adjust if needed
        cout << "N: " << x[index][0] << ", ";
        cout << "E: " << x[index][1] << ", ";
        cout << "D: " << x[index][2] << endl;
        
        //cout<<endl;
    }
    
    cout<<"end"<<endl;
}





//Function to get pqr at each time step:
double ffunc(double t,double &p,double &q, double &r){
 const double PI=3.1415926;
 p=PI/6;
 q=cos(t*6/PI);
 r=3*sin(t*30/PI);
 return 0;
}


//Function to get derivative:
void derivative_function(vector<double> x, vector<vector<double>> dcm, vector<double> v_body, double time,vector<double> &xdot,vector<vector<double>> &Cdot){
    double phi=x[0];
    double theta=x[1];
    double psi=x[2];

    double p,q,r;
    ffunc(time,p,q,r);
    cout << "p: "<<p<<"," << "q: "<<q<<"," << "r: "<<r<<endl;
    //Might need to change to a 3x3, with 0s filling empty spaces
    vector<double> pqr_vector={p,q,r};
    
    vector<vector<double>> pqr_matrix = {
        {p, 0.0, 0.0},
        {q, 0.0, 0.0},
        {r, 0.0, 0.0}
    };
    
    //Gimbal Eqn:
    vector<vector<double>> qdot(3, std::vector<double>(3));
    qdot[0][0] = 1;
    qdot[0][1] = tan(theta) * sin(phi);
    qdot[0][2] = tan(theta) * cos(phi);

    qdot[1][0] = 0;
    qdot[1][1] = cos(phi);
    qdot[1][2] = -sin(phi);

    qdot[2][0] = 0;
    qdot[2][1] = sin(phi) / cos(theta);
    qdot[2][2] = cos(phi) / cos(theta);

    vector<vector<double>> qdot_matrix;
    qdot_matrix=multiplyMatrices(qdot,pqr_matrix);

    //Strapdown:
    vector<vector<double>> strapdown={
        {0.0, -r, q},
        {r, 0.0, -p},
        {-q, p, 0.0}
    };
    
    //Cdot solving
    Cdot=multiplyMatrices(dcm,strapdown);

    vector<vector<double>> V_NED;
    //getting v_body into a useable form, MIGHT want to change this later( check if need row or column vector)
    vector<vector<double>> v_body_matrix={
        {v_body[0],0.0, 0.0},
        {v_body[1], 0.0, 0.0},
        {v_body[2], 0.0, 0.0}
    };
    //V_NED solving
    V_NED=multiplyMatrices(dcm,v_body_matrix);
    vector<double> V_NED_Used={V_NED[0][0],V_NED[1][0],V_NED[2][0]};
    //Filling xdot
    // Insert all elements from qdot into xdot
    for (const auto& row : qdot) {
        xdot.insert(xdot.end(), row.begin(), row.end());
    }

    // Add zero elements to xdot
    xdot.push_back(0.0);
    xdot.push_back(0.0);
    xdot.push_back(0.0);

    // Add V_NED elements to xdot
    xdot.insert(xdot.end(), V_NED_Used.begin(), V_NED_Used.end());
}

//Function to compute the dot product of two 3x1 vectors 
double dotProduct(const std::vector<double>& vec1, const std::vector<double>& vec2) {
   
    double result;

    // Calculate the dot product
    for (size_t i = 0; i < 3; ++i) {
        result += vec1[i] * vec2[i];
    }

    return result;
}

//Function to compute 3x3 matrix multiplication:
vector<vector<double>> multiplyMatrices(vector<vector<double>>& mat1, vector<vector<double>>& mat2) {
    // Initialize the result matrix with zeros
    vector<vector<double>> result(3, vector<double>(3, 0.0));

    // Perform matrix multiplication
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            for (size_t k = 0; k < 3; ++k) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }

    return result;
}

// Function to add two vectors
vector<double> addVectors( vector<double> a, vector<double> b) {
    //cout<<"addvectors"<<endl;
     // Check the size of vector a
    //std::cout << "Number of values in vector a: " << a.size() << std::endl;
     // Check the size of vector b
    //std::cout << "Number of values in vector b: " << b.size() << std::endl;
    vector<double> result(3);
    for (size_t i = 0; i < a.size(); ++i) {
        //cout<<"the vector b is:"<<endl;
        //cout<<b[i]<<endl;

        result[i] = a[i] + b[i];

        //cout<<"the vector result is:"<<endl;
        //cout<<result[i]<<endl;

        
    }
    return result;
}

// Function to multiply a vector by a scalar
vector<double> scaleVector(vector<double> vec, double scalar) {
    //cout<<"scalevectors"<<endl;
    vector<double> result(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        result[i] = vec[i] * scalar;
    }
    return result;
}
// Function to multiply a vector by a scalar
vector<vector<double>> scaleMatrix(vector<vector<double>> &vec, double scalar) {
    //cout<<"scalematrix"<<endl;
    //cout<<scalar<<endl;
    size_t rows = vec.size();
    // Get the number of columns (assuming at least one row)
    size_t cols = rows > 0 ? vec[0].size() : 0;
    //cout << "Number of rows: " << rows << endl;
    //cout << "Number of columns: " << cols << endl;

    vector<vector<double>> result(3, vector<double>(3));
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            
            //cout<<vec[i][j]<<endl;
        result[i][j] = vec[i][j] * scalar;
    }
    }
    //cout<<"scalematrix_FINISHED"<<endl;
    return result;
}
vector<vector<double>> addMatrix(vector<vector<double>> a, vector<vector<double>> b) {
    //cout<<"addmatrix"<<endl;
    vector<vector<double>> result(3, vector<double>(3));
    
    size_t rows = b.size();
    // Get the number of columns (assuming at least one row)
    size_t cols = rows > 0 ? b[0].size() : 0;
    //cout << "Number of rows: " << rows << endl;
    //cout << "Number of columns: " << cols << endl;
    
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            //cout<<"b"<<endl;
            //cout<<b[i][j]<<endl;
            //cout<<"a"<<endl;
            //cout<<a[i][j]<<endl;
        result[i][j] = a[i][j] + b[i][j];
        //cout<<result[i][j]<<endl;
       
    }
     //cout<<"addmatrixFINISHED"<<endl;
    }
    return result;
}

vector<double> add4Vectors( vector<double> a, vector<double> b, vector<double> c, vector<double> d) {
    vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] + b[i] + c[i] + d[i];
    }
    return result;
}

vector<vector<double>> add4Matrix(vector<vector<double>> a, vector<vector<double>> b, vector<vector<double>> c, vector<vector<double>> d) {
    vector<vector<double>> result(3, vector<double>(3));
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
        result[i][j] = a[i][j] + b[i][j] + c[i][j] + d[i][j];
    }
    }
    return result;
}