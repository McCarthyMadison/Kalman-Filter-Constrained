% Madison McCarthy (Using some code provided by D. Simon for PDF truncation)
% Combined Test for every constrained Application
% Kalman Filter with state constraints
clear all; clc; close all;

L = 50; % Runs for 50 time steps
dt = 1; % Time step =1.0 seconds

% Measurement Noise Variance
var = 90;

% Process Noise Variance
var_pos = 20;
var_vel = 2;

% Standard Kalman Filter Initialization (Test #1)

A_1 = [1 dt 0 0;0 1 0 0;0 0 1 dt;0 0 0 1]; %State transition matrix
B_1 = [0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0]; %control Matrix
u_1 = [0;0;0;0]; %Control vector
H_1 = [1 0 0 0;0 0 1 0]; %Measurement Matrix
I_1 = eye(4); %Identity matrix

Q_1 = diag([var_pos var_vel var_pos var_vel]); % Process noise covariance
R_1 = eye(2)*var; %Measurement noise covariance
P_1 = 100*eye(4); %Estimated covariance error


% Standard Kalman Filter Initialization (Test #2)

A_2 =A_1; %State transition matrix
B_2 = [0 0 0 0; 0 0 0 0;0 0 1 0;0 0 0 1]; %control matrix
u_2 = [0;0;0.5*4*dt^2;4*dt]; %control vector
H_2 = H_1; %Measurement Matrix
I_2 = I_1; %Identity matrix

Q_2 = Q_1; % Process noise covariance
R_2 = R_1; %Measurement noise covariance
P_2 = P_1; %Estimated covariance error


% Standard Kalman Filter Initialization (Test #3)

A_3 = A_1; %State transition matrix
B_3 = B_1; %control Matrix
u_3 = u_1; %Control vector
H_3 = H_1; %Measurement Matrix
I_3 = I_1; %Identity matrix 

Q_3 = Q_1; % Process noise covariance
R_3 = R_1; %Measurement noise covariance
P_3 = P_1; %Estimated covariance error



% Constraints
D_1 = [0 1 0 -3/4];
d_1 = 0;
D_3(1,:) = [0 0 1 0];
d_3(1) = 300;


%Perfect measurement method (Test #1 and Test #2)

% Test #1
H_perf_1 = [1 0 0 0;0 0 1 0;0 1 0 -3/4;];
R_perf_1 = diag([var var 0.00001]);
Q_perf_1 = Q_1;
P_perf_1 = P_1;

% Test #2
H_perf_2 = H_perf_1;
R_perf_2 = R_perf_1;
Q_perf_2 = Q_2;
P_perf_2 = P_2;


%State Reduction method (Test #1 and Test #2)

% Test #1
A_red_1 = [1 0 3/4*dt;0 1 dt;0 0 1];
B_red_1= [0 0 0;0 0 0;0 0 0];
u_red_1= [0;0;0];
H_red_1 = [1 0 0;0 1 0];
R_red_1 = diag([var var]);
Q_red_1 = diag([var_pos var_pos var_vel]);
P_red_1 = 100*eye(3);
I_red_1 = eye(3);

% Test #2
A_red_2 = A_red_1;
B_red_2= [0 0 0;0 1 0;0 0 1];
u_red_2= [0;0.5*4*dt^2;4*dt];
H_red_2 = H_red_1;
R_red_2 = R_red_1;
Q_red_2 = Q_red_1;
P_red_2 = P_red_1;
I_red_2 = I_red_1;


%Constrained Projection method (Test #1 and Test #3)

% Test #1
H_pro_1 = H_1;
R_pro_1 = R_1;
Q_pro_1 = Q_1;
P_pro_1 = P_1;

% Test #3
H_pro_3 = H_3;
R_pro_3 = R_3;
Q_pro_3 = Q_3;
P_pro_3 = P_3;

% PDF Truncation method (Test #1 and Test #3)
P_trun_1 = P_1;
P_trun_3 = P_3;


% Common Noise for all test cases
v = sqrt(var)*randn(2,L); %Creating measurement noise vector
pv(1,:) = sqrt(var_pos)*randn(1,L); %Creating process noise vector
pv(2,:) = sqrt(var_vel)*randn(1,L); %Creating process noise vector
pv(3,:) = sqrt(var_pos)*randn(1,L); %Creating process noise vector
pv(4,:) = sqrt(var_vel)*randn(1,L); %Creating process noise vector

%Initialization
x_estimated_1(:,1) = [200;50;-50;50]; %Initial state estimate
x_estimated_2(:,1) = x_estimated_1(:,1); 
x_estimated_3(:,1) = x_estimated_1(:,1); 

x_estimated_perf_1 = x_estimated_1;
x_estimated_perf_2 = x_estimated_2;

x_estimated_red_1(1,:) = x_estimated_1(1,:);
x_estimated_red_1(2,:) = x_estimated_1(3,:);
x_estimated_red_1(3,:) = x_estimated_1(4,:);

x_estimated_red_2(1,:) = x_estimated_2(1,:);
x_estimated_red_2(2,:) = x_estimated_2(3,:);
x_estimated_red_2(3,:) = x_estimated_2(4,:);

x_estimated_pro_1 = x_estimated_1;
x_estimated_pro_3 = x_estimated_3;

x_estimated_trun_1 = x_estimated_1;
x_estimated_trun_3 = x_estimated_3;


%Generating the system dynamics for 3 Test cases
m_velocity = 20;
x_actual_1(1,1) = 0; %X initial position
x_actual_1(2,1) = m_velocity*(3/4); %X velocity initial
x_actual_1(3,1) = 0; %Y initial position
x_actual_1(4,1) = m_velocity; %Y velocity initial

x_actual_2 = x_actual_1; %Same starting point for all 3 tests
x_actual_3 = x_actual_1;

% Test Scenario #1
for i=1:L-1
 
    % Producing the actual system dynamics (Test #1)
    x_actual_1(1,i+1) = x_actual_1(1,i) + x_actual_1(2,i)*dt;
    x_actual_1(2,i+1) = x_actual_1(2,i);
    
    x_actual_1(3,i+1) = x_actual_1(3,i) + x_actual_1(4,i)*dt;
    x_actual_1(4,i+1) = x_actual_1(4,i);
    
end

% Test scenario #2
for i=1:L/2
 
    % Producing the actual system dynamics (Test #2)
    x_actual_2(1,i+1) = x_actual_2(1,i) + x_actual_2(2,i)*dt;
    x_actual_2(2,i+1) = x_actual_2(2,i);
    
    x_actual_2(3,i+1) = x_actual_2(3,i) + x_actual_2(4,i)*dt + 0.5*4*dt^2;
    x_actual_2(4,i+1) = x_actual_2(4,i) + 4*dt;
    
end
for i=(L/2+1):L-1
 
    % Producing the actual system dynamics (Test #2)
    x_actual_2(1,i+1) = x_actual_2(1,i) + x_actual_2(2,i)*dt;
    x_actual_2(2,i+1) = x_actual_2(2,i);
    
    x_actual_2(3,i+1) = x_actual_2(3,i) + x_actual_2(4,i)*dt;
    x_actual_2(4,i+1) = x_actual_2(4,i);

end

% Test Scenairo #3
for i=1:L-1
 
    % Producing the actual system dynamics (Test #3)
    x_actual_3(1,i+1) = x_actual_3(1,i) + x_actual_3(2,i)*dt;
    x_actual_3(2,i+1) = x_actual_3(2,i);
    
    x_actual_3(3,i+1) = x_actual_3(3,i) + x_actual_3(4,i)*dt;
    x_actual_3(4,i+1) = x_actual_3(4,i);
    
     if (x_actual_3(3,i+1) > 300)
        x_actual_3(3,i+1) = 300;
        x_actual_3(4,i+1) = 0; 
     end
    
end

% Adding the process noise
x_actual_1 = x_actual_1 + pv;
x_actual_2 = x_actual_2 + pv;
x_actual_3 = x_actual_3 + pv;

%State reduction
x_actual_red_1(1,:) = x_actual_1(1,:);
x_actual_red_1(2,:) = x_actual_1(3,:);
x_actual_red_1(3,:) = x_actual_1(4,:);

x_actual_red_2(1,:) = x_actual_2(1,:);
x_actual_red_2(2,:) = x_actual_2(3,:);
x_actual_red_2(3,:) = x_actual_2(4,:);

% Perfect measurement
x_actual_perf_1 = x_actual_1;
x_actual_perf_2 = x_actual_2;

% Estimation Projection
x_actual_pro_1 = x_actual_1;
x_actual_pro_3 = x_actual_3;


%Creating the measured dynamics

% Test #1
z_1(1,:) = x_actual_1(1,:);
z_1(2,:) = x_actual_1(3,:);
z_1 = z_1 + v;

% Test #2
z_2(1,:) = x_actual_2(1,:);
z_2(2,:) = x_actual_2(3,:);
z_2 = z_2 + v;

% Test #3
z_3(1,:) = x_actual_3(1,:);
z_3(2,:) = x_actual_3(3,:);
z_3 = z_3 + v;

% State reduction
z_red_1 = z_1;
z_red_2 = z_2;

% Perfect measurement
z_perf_1 = z_1;
z_perf_1(3,:) = zeros(1,L);

z_perf_2 = z_2;
z_perf_2(3,:) = zeros(1,L);

%Estimation projection
z_pro_1 = z_1;
z_pro_3 = z_3;

%Kalman Tracking (Tests #1)
for i=1:(L)
    
    % Standard Kalman Filter for Test Scenario #1
    x_predicted_1 = A_1*x_estimated_1(:,i) + B_1*u_1; 
    P_predicted_1 = A_1*P_1*A_1'+Q_1;  
    y_tilda_1(:,i) = z_1(:,i) - H_1*x_predicted_1; 
    S_1 = H_1*P_predicted_1*H_1' + R_1; 
    K_1 = P_predicted_1*H_1'*inv(S_1); 
    x_estimated_1(:,i+1) = x_predicted_1 + K_1*y_tilda_1(:,i); 
    P_1 = (I_1 - K_1*H_1)*P_predicted_1; 
    
    
     % Perfect Measurement Method Kalman filter for Test Scenario #1
    x_predicted_perf_1 = A_1*x_estimated_perf_1(:,i) + B_1*u_1;
    P_predicted_perf_1 = A_1*P_perf_1*A_1'+Q_perf_1;
    y_tilda_perf_1(:,i) = z_perf_1(:,i) - H_perf_1*x_predicted_perf_1;
    S_perf_1 = H_perf_1*P_predicted_perf_1*H_perf_1' + R_perf_1;
    K_perf_1 = P_predicted_perf_1*H_perf_1'*inv(S_perf_1);
    x_estimated_perf_1(:,i+1) = x_predicted_perf_1 + K_perf_1*y_tilda_perf_1(:,i);
    P_perf_1 = (I_1 - K_perf_1*H_perf_1)*P_predicted_perf_1;
    
    % State Reduction Method Kalman filter for Test Scenario #1
    x_predicted_red_1 = A_red_1*x_estimated_red_1(:,i) + B_red_1*u_red_1;
    P_predicted_red_1 = A_red_1*P_red_1*A_red_1'+Q_red_1;
    y_tilda_red_1(:,i) = z_red_1(:,i) - H_red_1*x_predicted_red_1;
    S_red_1 = H_red_1*P_predicted_red_1*H_red_1' + R_red_1;
    K_red_1 = P_predicted_red_1*H_red_1'*inv(S_red_1);
    x_estimated_red_1(:,i+1) = x_predicted_red_1 + K_red_1*y_tilda_red_1(:,i);
    P_red_1 = (I_red_1 - K_red_1*H_red_1)*P_predicted_red_1;
    
    
    % Projection Method Kalman filter for Test Scenario #1
    x_predicted_pro_1 = A_1*x_estimated_pro_1(:,i) + B_1*u_1;
    P_predicted_pro_1 = A_1*P_pro_1*A_1'+Q_pro_1;
    y_tilda_pro_1(:,i) = z_pro_1(:,i) - H_pro_1*x_predicted_pro_1;
    S_pro_1 = H_pro_1*P_predicted_pro_1*H_pro_1' + R_pro_1;
    K_pro_1 = P_predicted_pro_1*H_pro_1'*inv(S_pro_1);
    x_estimated_pro_1(:,i+1) = x_predicted_pro_1 + K_pro_1*y_tilda_pro_1(:,i);
    P_pro_1 = (I_1 - K_pro_1*H_pro_1)*P_predicted_pro_1;
    
    % Projection onto constraint surface
    x_estimated_pro_1(:,i+1) = x_estimated_pro_1(:,i+1) - P_pro_1*D_1'*inv(D_1*P_pro_1*D_1')*(D_1*x_estimated_pro_1(:,i+1) - d_1); 
   
    
     % PDF Truncation Kalman Filter for Test Scenario #1
    x_predicted_trun_1 = A_1*x_estimated_trun_1(:,i) + B_1*u_1; 
    P_predicted_trun_1 = A_1*P_trun_1*A_1'+Q_1; 
    y_tilda_trun_1(:,i) = z_1(:,i) - H_1*x_predicted_trun_1; 
    S_trun_1 = H_1*P_predicted_trun_1*H_1' + R_1; 
    K_trun_1 = P_predicted_trun_1*H_1'*inv(S_trun_1); 
    x_estimated_trun_1(:,i+1) = x_predicted_trun_1 + K_trun_1*y_tilda_trun_1(:,i); 
    P_trun_1 = (I_1 - K_trun_1*H_1)*P_predicted_trun_1; 
    
 
     % Code Provided by D. Simon for PDF Truncation
     xTrunc_1 = x_estimated_trun_1(:,i+1);
     PTrunc_1 = P_trun_1;
     for k = 1 : 1
        [Utrunc_1, Wtrunc_1, Vtrunc_1] = svd(PTrunc_1);
        Ttrunc_1 = Utrunc_1;
        TTT_1 = Ttrunc_1 * Ttrunc_1';
       
        % Compute the modified Gram-Schmidt transformation S * Amgs = [ Wmgs ; 0 ].
        % Amgs is a given n x m matrix, and S is an orthogonal n x n matrix, and Wmgs is an m x m matrix.
        Amgs_1 = sqrt(Wtrunc_1) * Ttrunc_1' * D_1(k,:)'; % n x 1, where n = number of states
        [Wmgs_1, S_1] = MGS(Amgs_1);
        S_1 = S_1 * sqrt(D_1(k,:) * PTrunc_1 * D_1(k,:)') / Wmgs_1;
        cTrunc_1 = (d_1 - D_1(k,:) * xTrunc_1) / sqrt(D_1(k,:) * PTrunc_1 * D_1(k,:)');
        dTrunc_1 = (d_1 - D_1(k,:) * xTrunc_1) / sqrt(D_1(k,:) * PTrunc_1 * D_1(k,:)');
        
        % The following two lines are used for equality constraints.
        cTrunc_1 = (d_1 - D_1(k,:) * xTrunc_1) / sqrt(D_1(k,:) * PTrunc_1 * D_1(k,:)');
        mu_1 = cTrunc_1;
        sigma_1 = 0;
        
        zTrunc_1 = zeros(size(xTrunc_1));
        zTrunc_1(1) = mu_1;
        CovZ_1 = eye(length(zTrunc_1));
        CovZ_1(1,1) = sigma_1;
        xTrunc_1 = Ttrunc_1 * sqrt(Wtrunc_1) * S_1' * zTrunc_1 + xTrunc_1;
        PTrunc_1 = Ttrunc_1 * sqrt(Wtrunc_1) * S_1' * CovZ_1 * S_1 * sqrt(Wtrunc_1) * Ttrunc_1';
     end
     x_estimated_trun_1(:,i+1) = xTrunc_1';
     P_trun_1 = PTrunc_1';
    
    
end

%Kalman Tracking (Tests #2) 
for i=1:(L)
    
    % Standard Kalman Filter for Test Scenario #2
    x_predicted_2 = A_2*x_estimated_2(:,i) + B_2*u_2; 
    P_predicted_2 = A_2*P_2*A_2'+Q_2;  
    y_tilda_2(:,i) = z_2(:,i) - H_2*x_predicted_2;
    S_2 = H_2*P_predicted_2*H_2' + R_2;
    K_2 = P_predicted_2*H_2'*inv(S_2); 
    x_estimated_2(:,i+1) = x_predicted_2 + K_2*y_tilda_2(:,i);
    P_2 = (I_2 - K_2*H_2)*P_predicted_2; 
    
    % Perfect Measurement Method Kalman filter for Test Scenario #2
    x_predicted_perf_2 = A_2*x_estimated_perf_2(:,i) + B_2*u_2;
    P_predicted_perf_2 = A_2*P_perf_2*A_2'+Q_perf_2;
    y_tilda_perf_2(:,i) = z_perf_2(:,i) - H_perf_2*x_predicted_perf_2;
    S_perf_2 = H_perf_2*P_predicted_perf_2*H_perf_2' + R_perf_2;
    K_perf_2 = P_predicted_perf_2*H_perf_2'*inv(S_perf_2);
    x_estimated_perf_2(:,i+1) = x_predicted_perf_2 + K_perf_2*y_tilda_perf_2(:,i);
    P_perf_2 = (I_2 - K_perf_2*H_perf_2)*P_predicted_perf_2;
    
    % State Reduction Method Kalman filter for Test Scenario #2
    x_predicted_red_2 = A_red_2*x_estimated_red_2(:,i) + B_red_2*u_red_2;
    P_predicted_red_2 = A_red_2*P_red_2*A_red_2'+Q_red_2;
    y_tilda_red_2(:,i) = z_red_2(:,i) - H_red_2*x_predicted_red_2;
    S_red_2 = H_red_2*P_predicted_red_2*H_red_2' + R_red_2;
    K_red_2 = P_predicted_red_2*H_red_2'*inv(S_red_2);
    x_estimated_red_2(:,i+1) = x_predicted_red_2 + K_red_2*y_tilda_red_2(:,i);
    P_red_2 = (I_red_2 - K_red_2*H_red_2)*P_predicted_red_2;
       
    
    %Time varying constraint (Test #2)
    if (i <= L/2)
        H_perf_2 = [1 0 0 0;0 0 1 0;0 1 0 -15/(20+(i*4)-1)];
        A_red_2 = [1 0 15/(20+(i*4)-1);0 1 dt;0 0 1];
    end
    
     % Control Matrix changes for the system (Test #2)
    if (i == L/2)
        B_2 = [0 0 0 0; 0 0 0 0;0 0 0 0;0 0 0 0]; 
        B_red_2 = [0 0 0;0 0 0;0 0 0;];
    end
    
end

%Kalman Tracking (Tests #3) 
for i=1:(L)
    
    % Standard Kalman Filter for Test Scenario #3
    x_predicted_3 = A_3*x_estimated_3(:,i) + B_3*u_3; 
    P_predicted_3 = A_3*P_3*A_3'+Q_3;  
    y_tilda_3(:,i) = z_3(:,i) - H_3*x_predicted_3;
    S_3 = H_3*P_predicted_3*H_3' + R_3; 
    K_3 = P_predicted_3*H_3'*inv(S_3); 
    x_estimated_3(:,i+1) = x_predicted_3 + K_3*y_tilda_3(:,i);
    P_3 = (I_3 - K_3*H_3)*P_predicted_3; 
    
    
   
     % Projection Method Kalman filter for Test Scenario #3
    x_predicted_pro_3 = A_3*x_estimated_pro_3(:,i) + B_3*u_3;
    P_predicted_pro_3 = A_3*P_pro_3*A_3'+Q_pro_3;
    y_tilda_pro_3(:,i) = z_pro_3(:,i) - H_pro_3*x_predicted_pro_3;
    S_3 = H_pro_3*P_predicted_pro_3*H_pro_3' + R_pro_3;
    K_pro_3 = P_predicted_pro_3*H_pro_3'*inv(S_3);
    x_estimated_pro_3(:,i+1) = x_predicted_pro_3 + K_pro_3*y_tilda_pro_3(:,i);
    P_pro_3 = (I_3 - K_pro_3*H_pro_3)*P_predicted_pro_3;
    
    % Projection onto constraint surface (a-priori knowledge)
    if (i >= 16)
        x_estimated_pro_3(:,i+1) = x_estimated_pro_3(:,i+1) - P_pro_3*D_3'*inv(D_3*P_pro_3*D_3')*(D_3*x_estimated_pro_3(:,i+1) - d_3); 
    end
     
     
    
    
    % PDF truncation Kalman Filter for Test Scenario #3
    x_predicted_trun_3 = A_3*x_estimated_trun_3(:,i) + B_3*u_3; 
    P_predicted_trun_3 = A_3*P_trun_3*A_3'+Q_3;  
    y_tilda_trun_3(:,i) = z_3(:,i) - H_3*x_predicted_trun_3; 
    S_trun_3 = H_3*P_predicted_trun_3*H_3' + R_3; 
    K_trun_3 = P_predicted_trun_3*H_3'*inv(S_trun_3);
    x_estimated_trun_3(:,i+1) = x_predicted_trun_3 + K_3*y_tilda_trun_3(:,i);
    P_trun_3 = (I_3 - K_3*H_3)*P_predicted_trun_3; 
    
     % Code Provided by D. Simon for PDF Truncation
     xTrunc_3 = x_estimated_trun_3(:,i+1);
     PTrunc_3 = P_trun_3;
     for k = 1 : 1
        [Utrunc_3, Wtrunc_3, Vtrunc_3] = svd(PTrunc_3);
        Ttrunc_3 = Utrunc_3;
        TTT_3 = Ttrunc_3 * Ttrunc_3';
       
        % Compute the modified Gram-Schmidt transformation S * Amgs = [ Wmgs ; 0 ].
        % Amgs is a given n x m matrix, and S is an orthogonal n x n matrix, and Wmgs is an m x m matrix.
        Amgs_3 = sqrt(Wtrunc_3) * Ttrunc_3' * D_3(k,:)'; % n x 1, where n = number of states
        [Wmgs_3, S_3] = MGS(Amgs_3);
        S_3 = S_3 * sqrt(D_3(k,:) * PTrunc_3 * D_3(k,:)') / Wmgs_3;
        cTrunc_3 = (-999999 - D_3(k,:) * xTrunc_3) / sqrt(D_3(k,:) * PTrunc_3 * D_3(k,:)');
        dTrunc_3 = (d_3(k) - D_3(k,:) * xTrunc_3) / sqrt(D_3(k,:) * PTrunc_3 * D_3(k,:)');
        
        % The next 3 lines are for inequality constraints. In our example, they
        % are commented out because our problem uses equality constraints.
        alpha_3 = sqrt(2/pi) / (erf(dTrunc_3/sqrt(2)) - erf(cTrunc_3/sqrt(2)));
        mu_3 = alpha_3 * (exp(-cTrunc_3^2/2) - exp(-dTrunc_3^2/2));
        sigma_3 = alpha_3 * (exp(-cTrunc_3^2/2) * (cTrunc_3 - 2 * mu_3) - exp(-dTrunc_3^2/2) * (dTrunc_3 - 2 * mu_3)) + mu_3^2 + 1;
        
        zTrunc_3 = zeros(size(xTrunc_3));
        zTrunc_3(1) = mu_3;
        CovZ_3 = eye(length(zTrunc_3));
        CovZ_3(1,1) = sigma_3;
        xTrunc_3 = Ttrunc_3 * sqrt(Wtrunc_3) * S_3' * zTrunc_3 + xTrunc_3;
        PTrunc_3 = Ttrunc_3 * sqrt(Wtrunc_3) * S_3' * CovZ_3 * S_3 * sqrt(Wtrunc_3) * Ttrunc_3';
     end
     x_estimated_trun_3(:,i+1) = xTrunc_3';
     P_trun_3 = PTrunc_3';
    
end

% Determining actual error between states

error_1 = zeros(4,L);
error_2 = zeros(4,L);
error_3 = zeros(4,L);

error_perf_1 = zeros(4,L);
error_perf_2 = zeros(4,L);

error_ref_1 = zeros(3,L);
error_red_2 = zeros(3,L);

error_pro_1 = zeros(4,L);
error_pro_3 = zeros(4,L);

error_trun_1 = zeros(4,L);
error_trun_3 = zeros(4,L);

% Error for all three tests
for i=1:L
    
    for j=1:4
        
        error_1(j,i) = (x_estimated_1(j,i) - x_actual_1(j,i))/10^(floor(log10(abs(x_actual_1(j,i)))));
        error_2(j,i) = (x_estimated_2(j,i) - x_actual_2(j,i))/10^(floor(log10(abs(x_actual_2(j,i)))));
        error_3(j,i) = (x_estimated_3(j,i) - x_actual_3(j,i))/10^(floor(log10(abs(x_actual_3(j,i)))));

        error_perf_1(j,i) = (x_estimated_perf_1(j,i) - x_actual_perf_1(j,i))/10^(floor(log10(abs(x_actual_perf_1(j,i)))));
        error_perf_2(j,i) = (x_estimated_perf_2(j,i) - x_actual_perf_2(j,i))/10^(floor(log10(abs(x_actual_perf_2(j,i)))));
    
        error_pro_1(j,i) = (x_estimated_pro_1(j,i) - x_actual_pro_1(j,i))/10^(floor(log10(abs(x_actual_pro_1(j,i)))));
        error_pro_3(j,i) = (x_estimated_pro_3(j,i) - x_actual_pro_3(j,i))/10^(floor(log10(abs(x_actual_pro_3(j,i)))));

        error_trun_1(j,i) = (x_estimated_trun_1(j,i) - x_actual_1(j,i))/10^(floor(log10(abs(x_actual_1(j,i)))));
        error_trun_3(j,i) = (x_estimated_trun_3(j,i) - x_actual_3(j,i))/10^(floor(log10(abs(x_actual_3(j,i)))));
    end
   
    for j = 1:3
        error_red_1(j,i) = (x_estimated_red_1(j,i) - x_actual_red_1(j,i))/10^(floor(log10(abs(x_actual_red_1(j,i)))));
        error_red_2(j,i) = (x_estimated_red_2(j,i) - x_actual_red_2(j,i))/10^(floor(log10(abs(x_actual_red_2(j,i)))));      
    end
    
end

% Determining the MSE for the acual state error
actual_mse_1 = zeros(4,L);
actual_mse_2 = zeros(4,L);
actual_mse_3 = zeros(4,L);

actual_mse_1 = actual_mse_1 + error_1.^2;
actual_mse_2 = actual_mse_2 + error_2.^2;
actual_mse_3 = actual_mse_3 + error_3.^2;

actual_mse_1 = actual_mse_1/10;
actual_mse_2 = actual_mse_2/10;
actual_mse_3 = actual_mse_3/10;


actual_mse_perf_1 = zeros(4,L);
actual_mse_perf_2 = zeros(4,L);

actual_mse_red_1 = zeros(3,L);
actual_mse_red_2 = zeros(3,L);

actual_mse_pro_1 = zeros(4,L);
actual_mse_pro_3 = zeros(4,L);

actual_mse_trun_1 = zeros(4,L);
actual_mse_trun_3 = zeros(4,L);


actual_mse_perf_1 = actual_mse_perf_1 + error_perf_1.^2;
actual_mse_perf_2 = actual_mse_perf_2 + error_perf_2.^2;

actual_mse_red_1 = actual_mse_red_1 + error_red_1.^2;
actual_mse_red_2 = actual_mse_red_2 + error_red_2.^2;

actual_mse_pro_1 = actual_mse_pro_1 + error_pro_1.^2;
actual_mse_pro_3 = actual_mse_pro_3 + error_pro_3.^2;

actual_mse_trun_1 = actual_mse_trun_1 + error_trun_1.^2;
actual_mse_trun_3 = actual_mse_trun_3 + error_trun_3.^2;

actual_mse_perf_1 = actual_mse_perf_1/10;
actual_mse_perf_2 = actual_mse_perf_2/10;

actual_mse_red_1 = actual_mse_red_1/10;
actual_mse_red_2 = actual_mse_red_2/10;

actual_mse_pro_1 = actual_mse_pro_1/10;
actual_mse_pro_3 = actual_mse_pro_3/10;

actual_mse_trun_1 = actual_mse_trun_1/10;
actual_mse_trun_3 = actual_mse_trun_3/10;

% Test #1 Plot
%Plotting the actual state dynamics
figure(1)
plot(x_actual_1(1,:), x_actual_1(3,:));
hold on
% Plotting the measurements
plot(z_1(1,:),z_1(2,:),':');
%Plotting the estimation results (standard)
hold on
plot(x_estimated_1(1,:),x_estimated_1(3,:),'--');
%Plotting the estimation results (perfect measurement)
plot(x_estimated_perf_1(1,:),x_estimated_perf_1(3,:),'magenta');
%Plotting the estimation results (state reduction)
plot(x_estimated_red_1(1,:),x_estimated_red_1(2,:),'cyan');
%Plotting the estimation results (projection)
plot(x_estimated_pro_1(1,:),x_estimated_pro_1(3,:),'black');
%Plotting the estimation results (truncation)
plot(x_estimated_trun_1(1,:),x_estimated_trun_1(3,:),'green');
legend('true','observed','estimated','perfect','reduction', 'projection', 'truncation');
title('System Trajectory (Test #1)');
xlabel('X Position');ylabel('Y Position');

% Test #2 Plot
%Plotting the actual state dynamics
figure(2)
plot(x_actual_2(1,:), x_actual_2(3,:));
hold on
% Plotting the measurements
plot(z_2(1,:),z_2(2,:),':');
%Plotting the estimation results (standard)
hold on
plot(x_estimated_2(1,:),x_estimated_2(3,:),'--');
%Plotting the estimation results (perfect measurement)
plot(x_estimated_perf_2(1,:),x_estimated_perf_2(3,:),'magenta');
%Plotting the estimation results (state reduction)
plot(x_estimated_red_2(1,:),x_estimated_red_2(2,:),'cyan');
legend('true','observed','estimated','perfect', 'reduction');
title('System Trajectory (Test #2)');
xlabel('X Position');ylabel('Y Position');


% Test #3 Plot
%Plotting the actual state dynamics
figure(3)
plot(x_actual_3(1,:), x_actual_3(3,:));
hold on
% Plotting the measurements
plot(z_3(1,:),z_3(2,:),':');
%Plotting the estimation results (standard)
hold on
plot(x_estimated_3(1,:),x_estimated_3(3,:),'--');
%Plotting the estimation results (projection)
plot(x_estimated_pro_3(1,:),x_estimated_pro_3(3,:),'black');
%Plotting the estimation results (truncation)
plot(x_estimated_trun_3(1,:),x_estimated_trun_3(3,:),'green');
legend('true','observed','estimated', 'projection', 'truncation');
title('System Trajectory (Test #3)');
xlabel('X Position');ylabel('Y Position');


%Determining the MSE for the estimated states (objective function)
e_ensemble_kalman_1 = zeros(2,L);
e_ensemble_kalman_1 = e_ensemble_kalman_1 + y_tilda_1.^2; 

e_ensemble_kalman_2 = zeros(2,L);
e_ensemble_kalman_2 = e_ensemble_kalman_2 + y_tilda_2.^2; 

e_ensemble_kalman_3 = zeros(2,L);
e_ensemble_kalman_3 = e_ensemble_kalman_3 + y_tilda_3.^2; 
    
e_ensemble_kalman_perf_1 = zeros(3,L);
e_ensemble_kalman_perf_1 = e_ensemble_kalman_perf_1 + y_tilda_perf_1.^2; 

e_ensemble_kalman_perf_2 = zeros(3,L);
e_ensemble_kalman_perf_2 = e_ensemble_kalman_perf_2 + y_tilda_perf_2.^2; 

e_ensemble_kalman_red_1 = zeros(2,L);
e_ensemble_kalman_red_1 = e_ensemble_kalman_red_1 + y_tilda_red_1.^2; 

e_ensemble_kalman_red_2 = zeros(2,L);
e_ensemble_kalman_red_2 = e_ensemble_kalman_red_2 + y_tilda_red_2.^2; 

e_ensemble_kalman_pro_1 = zeros(2,L);
e_ensemble_kalman_pro_1 = e_ensemble_kalman_pro_1 + y_tilda_pro_1.^2; 

e_ensemble_kalman_pro_3 = zeros(2,L);
e_ensemble_kalman_pro_3 = e_ensemble_kalman_pro_3 + y_tilda_pro_3.^2; 

e_ensemble_kalman_trun_1 = zeros(2,L);
e_ensemble_kalman_trun_1 = e_ensemble_kalman_trun_1 + y_tilda_trun_1.^2; 

e_ensemble_kalman_trun_3 = zeros(2,L);
e_ensemble_kalman_trun_3 = e_ensemble_kalman_trun_3 + y_tilda_trun_3.^2; 

e_ensemble_kalman_1 = e_ensemble_kalman_1/10;
e_ensemble_kalman_2 = e_ensemble_kalman_2/10;
e_ensemble_kalman_3 = e_ensemble_kalman_3/10;

e_ensemble_kalman_perf_1 = e_ensemble_kalman_perf_1/10;
e_ensemble_kalman_perf_2 = e_ensemble_kalman_perf_2/10;

e_ensemble_kalman_red_1 = e_ensemble_kalman_red_1/10;
e_ensemble_kalman_red_2 = e_ensemble_kalman_red_2/10;

e_ensemble_kalman_pro_1 = e_ensemble_kalman_pro_1/10;
e_ensemble_kalman_pro_3 = e_ensemble_kalman_pro_3/10;

e_ensemble_kalman_trun_1 = e_ensemble_kalman_trun_1/10;
e_ensemble_kalman_trun_3 = e_ensemble_kalman_trun_3/10;


%Test #1 --- X position estimated MSE
figure(4)
plot(1:L,10*log10(e_ensemble_kalman_1(1,:)),'red');
m = mean(e_ensemble_kalman_1(1,:));
m = zeros(1,L) + m;
hold on

plot(1:L,10*log10(e_ensemble_kalman_perf_1(1,:)),'magenta');
m1 = mean(e_ensemble_kalman_perf_1(1,:));
m1 = zeros(1,L) + m1;

plot(1:L,10*log10(e_ensemble_kalman_red_1(1,:)),'cyan');
m2 = mean(e_ensemble_kalman_red_1(1,:));
m2 = zeros(1,L) + m2;

plot(1:L,10*log10(e_ensemble_kalman_pro_1(1,:)),'black');
m3 = mean(e_ensemble_kalman_pro_1(1,:));
m3 = zeros(1,L) + m3;

plot(1:L,10*log10(e_ensemble_kalman_trun_1(1,:)),'green');
m4 = mean(e_ensemble_kalman_trun_1(1,:));
m4 = zeros(1,L) + m4;

plot(1:L,10*log10(m/10),'red');
plot(1:L,10*log10(m1/10),'magenta');
plot(1:L,10*log10(m2/10),'cyan');
plot(1:L,10*log10(m3/10),'black');
plot(1:L,10*log10(m4/10),'green');

title('Test #1: Estimated MSE X Position');
legend('normal','perfect','reduction','projection','truncation');
xlabel('Time (samples)');ylabel('MSE (dB)');

fprintf('Results from Test #1 (X position):\n');
fprintf('estimated MSE (normal): %f\n', 10*log10(m(1)/10));
fprintf('estimated MSE (perfect): %f\n', 10*log10(m1(1)/10));
fprintf('estimated MSE (reduction): %f\n', 10*log10(m2(1)/10));
fprintf('estimated MSE (projection): %f\n', 10*log10(m3(1)/10));
fprintf('estimated MSE (truncation): %f\n\n', 10*log10(m4(1)/10));


%Test #1 --- X position actual MSE
figure(5)
plot(1:L,10*log10(actual_mse_1(1,:)),'red');
m = mean(actual_mse_1(1,:));
m = zeros(1,L) + m;
hold on

plot(1:L,10*log10(actual_mse_perf_1(1,:)),'magenta');
m1 = mean(actual_mse_perf_1(1,:));
m1 = zeros(1,L) + m1;

plot(1:L,10*log10(actual_mse_red_1(1,:)),'cyan');
m2 = mean(actual_mse_red_1(1,:));
m2 = zeros(1,L) + m2;

plot(1:L,10*log10(actual_mse_pro_1(1,:)),'black');
m3 = mean(actual_mse_pro_1(1,:));
m3 = zeros(1,L) + m3;

plot(1:L,10*log10(actual_mse_trun_1(1,:)),'green');
m4 = mean(actual_mse_trun_1(1,:));
m4 = zeros(1,L) + m4;

plot(1:L,10*log10(m/10),'red');
plot(1:L,10*log10(m1/10),'magenta');
plot(1:L,10*log10(m2/10),'cyan');
plot(1:L,10*log10(m3/10),'black');
plot(1:L,10*log10(m4/10),'green');

title('Test #1: Actual MSE X Position');
legend('normal','perfect','reduction','projection','truncation');
xlabel('Time (samples)');ylabel('MSE (dB)');

fprintf('actual MSE (normal): %f\n', 10*log10(m(1)/10));
fprintf('actual MSE (perfect): %f\n', 10*log10(m1(1)/10));
fprintf('actual MSE (reduction): %f\n', 10*log10(m2(1)/10));
fprintf('actual MSE (projection): %f\n', 10*log10(m3(1)/10));
fprintf('actual MSE (truncation): %f\n\n', 10*log10(m4(1)/10));


%Test #1 --- Y position estimated MSE
figure(6)
plot(1:L,10*log10(e_ensemble_kalman_1(2,:)),'red');
m = mean(e_ensemble_kalman_1(2,:));
m = zeros(1,L) + m;
hold on

plot(1:L,10*log10(e_ensemble_kalman_perf_1(2,:)),'magenta');
m1 = mean(e_ensemble_kalman_perf_1(2,:));
m1 = zeros(1,L) + m1;

plot(1:L,10*log10(e_ensemble_kalman_red_1(2,:)),'cyan');
m2 = mean(e_ensemble_kalman_red_1(2,:));
m2 = zeros(1,L) + m2;

plot(1:L,10*log10(e_ensemble_kalman_pro_1(2,:)),'black');
m3 = mean(e_ensemble_kalman_pro_1(2,:));
m3 = zeros(1,L) + m3;

plot(1:L,10*log10(e_ensemble_kalman_trun_1(2,:)),'green');
m4 = mean(e_ensemble_kalman_trun_1(2,:));
m4 = zeros(1,L) + m4;

plot(1:L,10*log10(m/10),'red');
plot(1:L,10*log10(m1/10),'magenta');
plot(1:L,10*log10(m2/10),'cyan');
plot(1:L,10*log10(m3/10),'black');
plot(1:L,10*log10(m4/10),'green');

title('Test #1: Estimated MSE Y Position');
legend('normal','perfect','reduction','projection','truncation');
xlabel('Time (samples)');ylabel('MSE (dB)');

fprintf('Results from Test #1 (Y position):\n');
fprintf('estimated MSE (normal): %f\n', 10*log10(m(1)/10));
fprintf('estimated MSE (perfect): %f\n', 10*log10(m1(1)/10));
fprintf('estimated MSE (reduction): %f\n', 10*log10(m2(1)/10));
fprintf('estimated MSE (projection): %f\n', 10*log10(m3(1)/10));
fprintf('estimated MSE (truncation): %f\n\n', 10*log10(m4(1)/10));


%Test #1 --- Y position actual MSE
figure(7)
plot(1:L,10*log10(actual_mse_1(3,:)),'red');
m = mean(actual_mse_1(3,:));
m = zeros(1,L) + m;
hold on

plot(1:L,10*log10(actual_mse_perf_1(3,:)),'magenta');
m1 = mean(actual_mse_perf_1(3,:));
m1 = zeros(1,L) + m1;

plot(1:L,10*log10(actual_mse_red_1(2,:)),'cyan');
m2 = mean(actual_mse_red_1(2,:));
m2 = zeros(1,L) + m2;

plot(1:L,10*log10(actual_mse_pro_1(3,:)),'black');
m3 = mean(actual_mse_pro_1(3,:));
m3 = zeros(1,L) + m3;

plot(1:L,10*log10(actual_mse_trun_1(3,:)),'green');
m4 = mean(actual_mse_trun_1(3,:));
m4 = zeros(1,L) + m4;

plot(1:L,10*log10(m/10),'red');
plot(1:L,10*log10(m1/10),'magenta');
plot(1:L,10*log10(m2/10),'cyan');
plot(1:L,10*log10(m3/10),'black');
plot(1:L,10*log10(m4/10),'green');

title('Test #1: Actual MSE Y Position');
legend('normal','perfect','reduction','projection','truncation');
xlabel('Time (samples)');ylabel('MSE (dB)');

fprintf('actual MSE (normal): %f\n', 10*log10(m(1)/10));
fprintf('actual MSE (perfect): %f\n', 10*log10(m1(1)/10));
fprintf('actual MSE (reduction): %f\n', 10*log10(m2(1)/10));
fprintf('actual MSE (projection): %f\n', 10*log10(m3(1)/10));
fprintf('actual MSE (truncation): %f\n\n', 10*log10(m4(1)/10));

%Test #2 --- X position estimated MSE
figure(8)
plot(1:L,10*log10(e_ensemble_kalman_2(1,:)),'red');
m = mean(e_ensemble_kalman_2(1,:));
m = zeros(1,L) + m;
hold on

plot(1:L,10*log10(e_ensemble_kalman_perf_2(1,:)),'magenta');
m1 = mean(e_ensemble_kalman_perf_2(1,:));
m1 = zeros(1,L) + m1;
hold on


plot(1:L,10*log10(e_ensemble_kalman_red_2(1,:)),'cyan');
m2 = mean(e_ensemble_kalman_red_2(1,:));
m2 = zeros(1,L) + m2;
hold on

plot(1:L,10*log10(m/10),'red');
plot(1:L,10*log10(m1/10),'magenta');
plot(1:L,10*log10(m2/10),'cyan');

title('Test #2: Estimated MSE X Position');
legend('normal','perf','reduction');
xlabel('Time (samples)');ylabel('MSE (dB)');

fprintf('Results from Test #2 (X position):\n');
fprintf('estimated MSE (normal): %f\n', 10*log10(m(1)/10));
fprintf('estimated MSE (perfect): %f\n', 10*log10(m1(1)/10));
fprintf('estimated MSE (reduction): %f\n\n', 10*log10(m2(1)/10));


%Test #2 --- X position actual MSE
figure(9)
plot(1:L,10*log10(actual_mse_2(1,:)),'red');
m = mean(actual_mse_2(1,:));
m = zeros(1,L) + m;
hold on


plot(1:L,10*log10(actual_mse_perf_2(1,:)),'magenta');
m1 = mean(actual_mse_perf_2(1,:));
m1 = zeros(1,L) + m1;


plot(1:L,10*log10(actual_mse_red_2(1,:)),'cyan');
m2 = mean(actual_mse_red_2(1,:));
m2 = zeros(1,L) + m2;


plot(1:L,10*log10(m/10),'red');
plot(1:L,10*log10(m1/10),'magenta');
plot(1:L,10*log10(m2/10),'cyan');

title('Test #2: Actual MSE X Position');
legend('normal','perfect','reduction');
xlabel('Time (samples)');ylabel('MSE (dB)');

fprintf('actual MSE (normal): %f\n', 10*log10(m(1)/10));
fprintf('actual MSE (perfect): %f\n', 10*log10(m1(1)/10));
fprintf('actual MSE (reduction): %f\n\n', 10*log10(m2(1)/10));

%Test #2 --- Y position estimated MSE
figure(10)
plot(1:L,10*log10(e_ensemble_kalman_2(2,:)),'red');
m = mean(e_ensemble_kalman_2(2,:));
m = zeros(1,L) + m;
hold on

plot(1:L,10*log10(e_ensemble_kalman_perf_2(2,:)),'magenta');
m1 = mean(e_ensemble_kalman_perf_2(2,:));
m1 = zeros(1,L) + m1;
hold on


plot(1:L,10*log10(e_ensemble_kalman_red_2(2,:)),'cyan');
m2 = mean(e_ensemble_kalman_red_2(2,:));
m2 = zeros(1,L) + m2;
hold on

plot(1:L,10*log10(m/10),'red');
plot(1:L,10*log10(m1/10),'magenta');
plot(1:L,10*log10(m2/10),'cyan');

title('Test #2: Estimated MSE Y Position');
legend('normal','perfect','reduction');
xlabel('Time (samples)');ylabel('MSE (dB)');

fprintf('Results from Test #2 (Y position):\n');
fprintf('estimated MSE (normal): %f\n', 10*log10(m(1)/10));
fprintf('estimated MSE (perfect): %f\n', 10*log10(m1(1)/10));
fprintf('estimated MSE (reduction): %f\n\n', 10*log10(m2(1)/10));


%Test #2 --- Y position actual MSE
figure(11)
plot(1:L,10*log10(actual_mse_2(3,:)),'red');
m = mean(actual_mse_2(3,:));
m = zeros(1,L) + m;
hold on


plot(1:L,10*log10(actual_mse_perf_2(3,:)),'magenta');
m1 = mean(actual_mse_perf_2(3,:));
m1 = zeros(1,L) + m1;


plot(1:L,10*log10(actual_mse_red_2(2,:)),'cyan');
m2 = mean(actual_mse_red_2(2,:));
m2 = zeros(1,L) + m2;


plot(1:L,10*log10(m/10),'red');
plot(1:L,10*log10(m1/10),'magenta');
plot(1:L,10*log10(m2/10),'cyan');

title('Test #2: Actual MSE Y Position');
legend('normal','perfect','reduction');
xlabel('Time (samples)');ylabel('MSE (dB)');

fprintf('actual MSE (normal): %f\n', 10*log10(m(1)/10));
fprintf('actual MSE (perfect): %f\n', 10*log10(m1(1)/10));
fprintf('actual MSE (reduction): %f\n\n', 10*log10(m2(1)/10));

%Test #3 --- X position estimated MSE
figure(12)
plot(1:L,10*log10(e_ensemble_kalman_3(1,:)),'red');
m = mean(e_ensemble_kalman_3(1,:));
m = zeros(1,L) + m;
hold on

plot(1:L,10*log10(e_ensemble_kalman_pro_3(1,:)),'black');
m3 = mean(e_ensemble_kalman_pro_3(1,:));
m3 = zeros(1,L) + m3;

plot(1:L,10*log10(e_ensemble_kalman_trun_3(1,:)),'green');
m4 = mean(e_ensemble_kalman_trun_3(1,:));
m4 = zeros(1,L) + m4;

plot(1:L,10*log10(m/10),'red');
plot(1:L,10*log10(m3/10),'black');
plot(1:L,10*log10(m4/10),'green');

title('Test #3: Estimated MSE X Position');
legend('normal','projection','truncation');
xlabel('Time (samples)');ylabel('MSE (dB)');

fprintf('Results from Test #3 (X position):\n');
fprintf('estimated MSE (normal): %f\n', 10*log10(m(1)/10));
fprintf('estimated MSE (projection): %f\n', 10*log10(m3(1)/10));
fprintf('actual MSE (truncation): %f\n\n', 10*log10(m4(1)/10));

%Test #3 --- X position actual MSE
figure(13)
plot(1:L,10*log10(actual_mse_3(1,:)),'red');
m = mean(actual_mse_3(1,:));
m = zeros(1,L) + m;
hold on

plot(1:L,10*log10(actual_mse_pro_3(1,:)),'black');
m3 = mean(actual_mse_pro_3(1,:));
m3 = zeros(1,L) + m3;

plot(1:L,10*log10(actual_mse_trun_3(1,:)),'green');
m4 = mean(actual_mse_trun_3(1,:));
m4 = zeros(1,L) + m4;

plot(1:L,10*log10(m/10),'red');
plot(1:L,10*log10(m3/10),'black');
plot(1:L,10*log10(m4/10),'green');

title('Test #3: Actual MSE X Position');
legend('normal','projection','truncation');
xlabel('Time (samples)');ylabel('MSE (dB)');

fprintf('actual MSE (normal): %f\n', 10*log10(m(1)/10));
fprintf('actual MSE (projection): %f\n', 10*log10(m3(1)/10));
fprintf('actual MSE (truncation): %f\n\n', 10*log10(m4(1)/10));

%Test #3 --- Y position estimated MSE
figure(14)
plot(1:L,10*log10(e_ensemble_kalman_3(2,:)),'red');
m = mean(e_ensemble_kalman_3(2,:));
m = zeros(1,L) + m;
hold on

plot(1:L,10*log10(e_ensemble_kalman_pro_3(2,:)),'black');
m3 = mean(e_ensemble_kalman_pro_3(2,:));
m3 = zeros(1,L) + m3;

plot(1:L,10*log10(e_ensemble_kalman_trun_3(2,:)),'green');
m4 = mean(e_ensemble_kalman_trun_3(2,:));
m4 = zeros(1,L) + m4;

plot(1:L,10*log10(m/10),'red');
plot(1:L,10*log10(m3/10),'black');
plot(1:L,10*log10(m4/10),'green');

title('Test #3: Estimated MSE Y Position');
legend('normal','projection','truncation');
xlabel('Time (samples)');ylabel('MSE (dB)');

fprintf('Results from Test #3 (Y position):\n');
fprintf('estimated MSE (normal): %f\n', 10*log10(m(1)/10));
fprintf('estimated MSE (projection): %f\n', 10*log10(m3(1)/10));
fprintf('actual MSE (truncation): %f\n\n', 10*log10(m4(1)/10));

%Test #3 --- Y position actual MSE
figure(15)
plot(1:L,10*log10(actual_mse_3(3,:)),'red');
m = mean(actual_mse_3(3,:));
m = zeros(1,L) + m;
hold on

plot(1:L,10*log10(actual_mse_pro_3(3,:)),'black');
m3 = mean(actual_mse_pro_3(3,:));
m3 = zeros(1,L) + m3;

plot(1:L,10*log10(actual_mse_trun_3(3,:)),'green');
m4 = mean(actual_mse_trun_3(3,:));
m4 = zeros(1,L) + m4;

plot(1:L,10*log10(m/10),'red');
plot(1:L,10*log10(m3/10),'black');
plot(1:L,10*log10(m4/10),'green');

title('Test #3: Actual MSE Y Position');
legend('normal','projection','truncation');
xlabel('Time (samples)');ylabel('MSE (dB)');

fprintf('actual MSE (normal): %f\n', 10*log10(m(1)/10));
fprintf('actual MSE (projection): %f\n', 10*log10(m3(1)/10));
fprintf('actual MSE (truncation): %f\n\n', 10*log10(m4(1)/10));
