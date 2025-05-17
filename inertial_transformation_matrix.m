%% Inertial Transformation Matrix, matrix_T
function T = inertial_transformation_matrix(alpha, beta, gamma) 

    T = [(cos(alpha)*cos(beta)), ((cos(alpha)*sin(beta)*sin(gamma))-(sin(alpha)*cos(gamma))), ((cos(alpha)*sin(beta)*cos(gamma))+(sin(alpha)*sin(gamma)));
     (sin(alpha)*cos(beta)), ((sin(alpha)*sin(beta)*sin(gamma))+(cos(alpha)*cos(gamma))), ((sin(alpha)*sin(beta)*cos(gamma))-(cos(alpha)*sin(gamma))); 
     -(sin(beta)), (cos(beta)*sin(gamma)), (cos(beta)*cos(gamma))];





end