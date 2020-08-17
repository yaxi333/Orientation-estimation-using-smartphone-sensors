function [x, P] = mu_m(x, P, mag, m0, Rm)
% implement the magnetometer measurement update
% mag: represent y^m_k model
% Rm:  measurement noise covariance matrix
% magnetometer measure model: y_k = Q'(q_k)(m0 + f_k) + e_k
% where f_k = 0

h = Qq(x)' * m0;

% calculate quaternion derivative
[Q0, Q1, Q2, Q3] = dQqdq(x);
% calculate derivative of function h
dh = [Q0' * m0, Q1' * m0, Q2' * m0, Q3' * m0];

S = dh * P * dh' + Rm;
K = P * dh' / S;

% update
x = x + K * (mag - h);
P = P - K * S * K';

end