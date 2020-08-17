function  [x, P] = mu_g(x, P, yacc, Ra, g0)
    % EKF update using accelerometer measurements
    % accelerometer model: y_k = Q'(q_k)(g0 + f_k) + e_k
    % Ra:  measurement noise covariance matrix
    % where f_k = 0
    
    % Measurement estimates
    hx = Qq(x)'*g0;
    
    % Derivatives of Q
    [Q0, Q1, Q2, Q3] = dQqdq(x);
    
    % Jacobian matrix 
    Hx = [Q0'*g0 Q1'*g0 Q2'*g0 Q3'*g0];
    
    % Innovation covariance
    Sk = Hx*P*Hx'+Ra;
    
    % Kalman gain
    Kk = P*Hx'/Sk;
    
    % Update x and P
    x = x + Kk*(yacc-hx);
    P = P-Kk*Sk*Kk';   
end