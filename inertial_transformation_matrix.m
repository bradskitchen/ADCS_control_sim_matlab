%% Inertial Transformation Matrix, matrix_T
function T = inertial_transformation_matrix(alpha, beta, gamma) 
    %p,t,s -> a,b,g
    T = [(cos(beta)*cos(gamma)), ((sin(alpha)*sin(beta)*cos(gamma))-(cos(alpha)*sin(gamma))), ((cos(alpha)*sin(beta)*cos(gamma))+(sin(alpha)*sin(gamma)));
     (cos(beta)*sin(gamma)), ((sin(alpha)*sin(beta)*sin(gamma))+(cos(alpha)*cos(gamma))), ((cos(alpha)*sin(beta)*sin(gamma))-(sin(alpha)*cos(gamma))); 
     -(sin(beta)), (sin(alpha)*cos(beta)), (cos(alpha)*cos(beta))];
end