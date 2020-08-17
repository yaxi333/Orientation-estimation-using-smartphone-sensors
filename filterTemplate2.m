function [xhat, meas] = filterTemplate2(calAcc, calGyr, calMag)
% FILTERTEMPLATE  Filter template
%
% This is a template function for how to collect and filter data
% sent from a smartphone live.  Calibration data for the
% accelerometer, gyroscope and magnetometer assumed available as
% structs with fields m (mean) and R (variance).
%
% The function returns xhat as an array of structs comprising t
% (timestamp), x (state), and P (state covariance) for each
% timestamp, and meas an array of structs comprising t (timestamp),
% acc (accelerometer measurements), gyr (gyroscope measurements),
% mag (magnetometer measurements), and orint (orientation quaternions
% from the phone).  Measurements not availabe are marked with NaNs.
%
% As you implement your own orientation estimate, it will be
% visualized in a simple illustration.  If the orientation estimate
% is checked in the Sensor Fusion app, it will be displayed in a
% separate view.
%
% Note that it is not necessary to provide inputs (calAcc, calGyr, calMag).

%% Setup necessary infrastructure
import('com.liu.sensordata.*');  % Used to receive data.

%% Filter settings
t0 = [];  % Initial time (initialize on first data received)
nx = 4;   % Assuming that you use q as state variable.
%%%%%%% Filter settings %%%%%%%%
% S7 Calibration matrices
% Rw = diag([2.285e-06 1.231e-06 1.729e-06]);
% Ra = diag([9.946e-05 1.163e-04 5.608e-05]);
% Rm = diag([0.152 0.0658 0.309]);
% g0 = [-0.040 0.189 9.823]'; 
% m0 = [0 17.517 -48.080]';

  % Add your filter settings here.
  Rm = diag([0.0624818624189320,0.0943352423095654,0.0952209540668264]);
  Rw = diag([1.15398198100157e-06,5.68145204434471e-06,3.78699308913974e-07]);
  Ra = diag([0.000220761712947003,9.51335118857420e-05,9.37980782424480e-05]);
  % Magnetometers bias: m0 = [0 sqrt(m_x^2+m_y^2) m_z]'   
  m0 = [0 10.4582 -54.5993].';
  % Accelerometer biases
  g0 = [0.1189 -0.0243 9.8919]'; 


% Other parameters
magOut = 1;
accOut = 1;
alpha = 0.01;
Lk = norm(m0);

% Current filter state.
x = [1; 0; 0 ;0];
P = eye(nx, nx);

% Saved filter states.
xhat = struct('t', zeros(1, 0),...
    'x', zeros(nx, 0),...
    'P', zeros(nx, nx, 0));

meas = struct('t', zeros(1, 0),...
    'acc', zeros(3, 0),...
    'gyr', zeros(3, 0),...
    'mag', zeros(3, 0),...
    'orient', zeros(4, 0));
try
    %% Create data link
    server = StreamSensorDataReader(3400);
    % Makes sure to resources are returned.
    sentinel = onCleanup(@() server.stop());
    
    server.start();  % Start data reception.
    
    % Used for visualization.
    figure(1);
    subplot(1, 2, 1);
    ownView = OrientationView('Own filter', gca);  % Used for visualization.
    googleView = [];
    counter = 0;  % Used to throttle the displayed frame rate.
    
    %% Filter loop
    while server.status()  % Repeat while data is available
        % Get the next measurement set, assume all measurements
        % within the next 5 ms are concurrent (suitable for sampling
        % in 100Hz).
        data = server.getNext(5);
        
        if isnan(data(1))  % No new data received
            continue;        % Skips the rest of the look
        end
        t = data(1)/1000;  % Extract current time
        
        if isempty(t0)  % Initialize t0
            t0 = t;
        end
        
        gyr = data(1, 5:7)';
        if ~any(isnan(gyr))  % Gyro measurements are available.
            [x, P] = tu_qw(x, P, gyr, t-t0-meas.t(end), Rw); % Update state estimate
            [x, P] = mu_normalizeQ(x,P); % Normalize state vector
        else
            % If no measurements are available assume random walk model
            P = P + 0.001*eye(4);
        end

        acc = data(1, 2:4)';
        if ~any(isnan(acc))  % Acc measurements are available.
            % If the 2-norm for acc measurement is to large/small, skip
            % update step
            if  norm(acc)<9.81*1.25 && norm(acc)>9.81*0.75
                [x, P] = mu_g(x, P, acc, Ra, g0); % Update state estimate
                [x, P] = mu_normalizeQ(x,P); % Normalize state vector
                accOut = 0;
            else
                accOut = 1;
            end
        end

        mag = data(1, 8:10)';
        if ~any(isnan(mag))  % Mag measurements are available.
            % AR-filter to account for that the magnitude of m0 might drift
            Lk = (1-alpha)*Lk + alpha*norm(mag);
            
            % If magnitude of measurement is too large, skip update step
            if 35<Lk && Lk<55 % Thresholds for magnetic field 
                [x, P] = mu_m(x, P, mag, m0, Rm); % Update state estimate
                [x, P] = mu_normalizeQ(x,P); % Normalize state vector
                magOut = 0;
            else
                magOut = 1;
            end
        end
        
        orientation = data(1, 18:21)';  % Google's orientation estimate.
        % Visualize result
        if rem(counter, 10) == 0
            setOrientation(ownView, x(1:4));
            ownView.setAccDist(accOut);
            ownView.setMagDist(magOut);
            title(ownView, 'OWN', 'FontSize', 16);
            if ~any(isnan(orientation))
                if isempty(googleView)
                    subplot(1, 2, 2);
                    % Used for visualization.
                    googleView = OrientationView('Google filter', gca);
                end
                setOrientation(googleView, orientation);
                title(googleView, 'GOOGLE', 'FontSize', 16);
            end
        end
        counter = counter + 1;
        
        % Save estimates
        xhat.x(:, end+1) = x;
        xhat.P(:, :, end+1) = P;
        xhat.t(end+1) = t - t0;
        
        meas.t(end+1) = t - t0;
        meas.acc(:, end+1) = acc;
        meas.gyr(:, end+1) = gyr;
        meas.mag(:, end+1) = mag;
        meas.orient(:, end+1) = orientation;
    end
catch e
    fprintf(['Unsuccessful connecting to client!\n' ...
        'Make sure to start streaming from the phone *after*'...
        'running this function!']);
end
end
