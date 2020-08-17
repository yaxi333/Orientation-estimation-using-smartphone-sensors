function [x, P] = tu_qw(x, P, omega, T, Rw)
% time update function that angular rate measurement available
% INPUT
%     x: mean for time k-1
%     P: covariance for time k-1
% omega: measured angular rate
%     T: the time since the last measurement
%    Rw: process noise covariance matrix
% OUTPUT
%     x: predict mean value for time k
%     P: predict covariance for time k

% calculate the discretized model
F = eye(size(x, 1)) + (T/2) * Somega(omega);
G = (T/2) * Sq(x);
% update
x = F * x;
P = F * P * F' + G * Rw * G';


end