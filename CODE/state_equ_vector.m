function state_equ = state_equ_vector(t, state_vector, K1, K2, q_final, state_initial)

    q1 = state_vector(1);
    q2 = state_vector(2);
    q1_dot = state_vector(3);
    q2_dot = state_vector(4);
    int_e1 = state_vector(5);  
    int_e2 = state_vector(6);  
    
    q_dot = [q1_dot; q2_dot];
    
    Kp1 = K1(1); 
    Kd1 = K1(2); 
    Ki1 = K1(3);
    Kp2 = K2(1); 
    Kd2 = K2(2); 
    Ki2 = K2(3);

    e1 = q_final(1) - q1;
    e2 = q_final(2) - q2;
    e1_dot = -q1_dot;  
    e2_dot = -q2_dot;  
    tau1 = Kp1*e1 + Kd1*e1_dot + Ki1*int_e1;
    tau2 = Kp2*e2 + Kd2*e2_dot + Ki2*int_e2;
    TAU = [tau1; tau2];
    m1 = 5;    
    m2 = 3;    
    l1 = 0.25; 
    l2 = 0.15; 
    g = 9.81;  

    M11 = (m1 + m2)*(l1^2) + m2*l2*(l2 + 2*l1*cos(q2));
    M12 = m2*l2*(l2 + l1*cos(q2));
    M21 = m2*l2*(l2 + l1*cos(q2));
    M22 = m2*l2*l2;
    M = [M11, M12; M21, M22];

    C11 = -m2*l1*l2*sin(q2)*q2_dot;
    C12 = -m2*l1*l2*sin(q2)*(q2_dot + q1_dot);
    C21 = 0;
    C22 = m2*l1*l2*sin(q2)*q1_dot;
    C = [C11, C12; C21, C22];
    
    G = [m1*l1*g*cos(q1) + m2*g*(l2*cos(q1 + q2) + l1*cos(q1)); 
         m2*g*l2*cos(q1 + q2)];
    
    q_dotdot = (M ^(-1))*(TAU - C*q_dot - G);
    state_equ = [q_dot; q_dotdot; e1; e2];        
    
end