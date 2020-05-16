//===========
BLA::Matrix<2,1> invkin(float xvel,float yvel,float rotvel, float alpha, float gamma){
    BLA::Matrix<2,3> Jbi = {1, 0, -D*sin(gamma+psi),
                            0, 1, D*cos(gamma+psi)};

    float C_1;
    float C_2;
    float C_3;
    float C_4;
    float wheelvel[2];
    
    //Jw*Ri*Rb 
    C_1 = ( (cos(alpha) - Lratio*sin(alpha)/2) * cos(psi) ) + ( (sin(alpha) + Lratio*cos(alpha)/2) * -sin(psi) ); 
    C_2 = ( (cos(alpha) - Lratio*sin(alpha)/2) * sin(psi) ) + ( (sin(alpha) + Lratio*cos(alpha)/2) * cos(psi) );
    C_3 = ( (cos(alpha) + Lratio*sin(alpha)/2) * cos(psi) ) + ( (sin(alpha) - Lratio*cos(alpha)/2) * -sin(psi) ); 
    C_4 = ( (cos(alpha) + Lratio*sin(alpha)/2) * sin(psi) ) + ( (sin(alpha) - Lratio*cos(alpha)/2) * cos(psi) );

    BLA::Matrix<2,2> C = {C_1, C_2,
                          C_3, C_4};

    BLA::Matrix<3,1> vel_body = {xvel,
                                 yvel,
                                 rotvel};                          

    BLA::Matrix<2,1> wheel_vel =  C*Jbi*vel_body;

    return wheel_vel;
}
