function [xhat, meas] = filterTemplate(calAcc, calGyr, calMag)
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
%     % Add your filter settings here.
% 
%     Rw = 1e-5 * [0.081733901510542   0.001723173871440  -0.002582723004641
%                  0.001723173871440   0.105466716020954   0.000782168729699
%                  -0.002582723004641   0.000782168729699   0.061062268681170]*10000;
%     
%     g0 = -[0.0171 -0.0403 -9.9719].';
%     Ra =    1.0e-03 *[0.1356   -0.0005    0.0001
%                       -0.0005    0.1441   -0.0066
%                       0.0001   -0.0066    0.2921]*1000;
%                   
%     m0 = [0   23.1169  -35.2728].';
%     Rm =[1.6583   -0.1538    0.0401
%          -0.1538    2.1136   -0.0003
%          0.0401   -0.0003    1.7076];
  load Mean.mat
  load Cov.mat 
% Add your filter settings here.
  Rm = diag([Cov(3,1), Cov(3,2), Cov(3,3)]);
  Rw = diag([Cov(2,1), Cov(2,2), Cov(2,3)]);
  Ra = diag([Cov(1,1), Cov(1,2), Cov(1,3)]);
  % Magnetometers bias: m0 = [0 sqrt(m_x^2+m_y^2) m_z]'   
  m0 = [0 sqrt(Mean(1,3).^2+Mean(2,3).^2) Mean(3,3)].';
  % Accelerometer biases
  g0 = [Mean(1,1) Mean(2,1) Mean(3,1)]'; 
    alpha = 0.001;
    L = alpha*norm(m0);
    
     
    % Current filter state.
    x = [1; 0; 0 ;0];
    P = eye(nx, nx);

    % Saved filter states.
    xhat = struct('t', zeros(1, 0),...
                'x', zeros(nx, 0),...
                'P', zeros(nx, nx, 0),...
                 'L', L);
    

    meas = struct('t', zeros(1, 0),...
                'acc', zeros(3, 0),...
                'gyr', zeros(3, 0),...
                'mag', zeros(3, 0),...
                'orient', zeros(4, 0));
    try
    % Create data link
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

    % Filter loop
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

      acc = data(1, 2:4)';
      acc_outlier = false;
      if ~any(isnan(acc))  % Acc measurements are available.
        [x, P, acc_outlier] = mu_g(x, P, acc, Ra, g0);
      end
      
        gyr = data(1, 5:7)';
        if ~any(isnan(gyr))  % Gyro measurements are available. 
            [x,P] = tu_qw(x, P, gyr, t-t0-meas.t(end), Rw);
        elseif ~size(meas.gyr,2) == 0 && ~any(isnan(meas.gyr(:, end)))
            [x,P] = tu_qw(x, P, meas.gyr(:, end), t-t0-meas.t(end), Rw); 
        end

        mag = data(1, 8:10)';
        mag_outlier = false;
        if ~any(isnan(mag))  % Mag measurements are available.
            [x, P, L, mag_outlier] = mu_m(x, P, mag, m0, Rm, xhat.L(:, end), alpha);
            xhat.L(:, end+1) = L;
        end

      orientation = data(1, 18:21)';  % Google's orientation estimate.

      % Visualize result
      if rem(counter, 10) == 0
        setOrientation(ownView, x(1:4));
        title(ownView, 'OWN', 'FontSize', 16);
        
        ownView.setAccDist(acc_outlier)
        ownView.setMagDist(mag_outlier)
        
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
    %fprintf(['Unsuccessful connecting to client!\n' ...
      %'Make sure to start streaming from the phone *after*'...
             %'running this function!']);
     e
    end
    
end





