u = 1;      %External force
M1 = 100;   % Mass 1
M2 = 100;   % Mass 2
k1 = 5;     % Spring constant 1
k2 = 50;    % Spring constant 2
k3 = 5;     % Spring constant 3
f1 = 100;   % Friction coefficient 1
f2 = 100;   % Friction coefficient 2

B1 = tf(1, [M1 0 0]);
B2 = tf(k1, 1);
B3 = tf(k2, 1);
B4 = tf([f1 0], 1);
B5 = tf(k2, 1);
B6 = tf(1, [M2 0 0]);
B7 = tf(k2, 1);
B8 = tf([f2 0], 1);
B9 = tf(k3, 1);
B10 = tf(k2, 1);

BlockMat = append(B1, B2, B3, B4, B5, B6, B7, B8, B9, B10);
connect_map = [  1, 10, -2, -3, -4;    2, 1, 0, 0, 0;    3, 1, 0, 0, 0;    4, 1, 0, 0, 0;    5, 1, 0, 0, 0;    6, 5, -7, -8, -9;    7, 6, 0, 0, 0;    8, 6, 0, 0, 0;    9, 6, 0, 0, 0;    10, 6, 0, 0, 0;];

%TF of (x1/u)
sys1 = connect(BlockMat, connect_map, 1, 1);
p1= stepplot(sys1);
pzmap (sys1);
[wn1,z1]=damp(sys1);
X1U_tf = sys1(1);
[num1, den1] = tfdata(X1U_tf, 'v');
fprintf('The value of X1/U transfer function is: \n');
fprintf('numerator: %s \n', mat2str(num1));
fprintf('denominator: %s \n', mat2str(den1));
%check stability for x1/u
poles_X1U = pole(X1U_tf);
if real(poles_X1U) < 0
    disp('The system is stable')
else
    disp('The system is unstable')
end

% Simulate the system for X1
t = 0:0.01:120;
X1 = lsim(sys1, u*ones(size(t)), t);
% Plot the responses
figure();
plot(t, X1);
xlabel('Time (s)');
ylabel('X1 (m)');
title('Response of X1 to a Fixed Input Force of 1N');

%TF of (x1/u)
sys2 = connect(BlockMat, connect_map, 1, 6);
figure();

p2= stepplot(sys2);
pzmap(sys2);
[wn2,z2]=damp(sys2);
X2U_tf = sys2(1);
[num2, den2] = tfdata(X2U_tf, 'v');
fprintf('The value of X2/U transfer function is: \n');
fprintf('numerator: %s \n', mat2str(num2));
fprintf('denominator: %s \n', mat2str(den2));
%check stability for x2/u
poles_X2U = pole(X2U_tf);
if real(poles_X2U) < 0
    disp('The system is stable')
else
    disp('The system is unstable')
end
% Simulate the system for X2  due to u=1
t = 0:0.01:120;
X2 = lsim(sys2, u*ones(size(t)), t);
figure();

plot(t, X2);
xlabel('Time (s)');
ylabel('X2 (m)');
title('Response of X2 to a Fixed Input Force of 1N');

% Calculate the steady state values
X1_ss = mean(X1(end-100:end));
X2_ss = mean(X2(end-100:end));
fprintf('The steady state value of X1 is %f m\n', X1_ss);
fprintf('The steady state value of X2 is %f m\n', X2_ss);

%req 5,req6,req7

% Define the closed-loop transfer function
closed_loop_tf = feedback(sys2, 1);

% Simulate the system for X1
t = 0:0.01:100;
Xd = ones(size(t))*2 ; % Desired displacement
X1 = lsim(closed_loop_tf, Xd, t);

% Plot the responses
figure();
 
plot(t, X1);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Response of the System to input xd=2m');
legend('X1', 'Xd');

% Calculate the rise time, peak time, maximum peak, settling time, and ess
step_info =stepinfo(closed_loop_tf*2);
rise_time = step_info.RiseTime;
peak_time = step_info.PeakTime;
max_peak = step_info.Peak;
settling_time = step_info.SettlingTime;
% Calculate the ess as the difference between the desired signal and the actual signal
ess = Xd(end) - X1(end);

% Display the results
fprintf('Rise Time: %.2f s\n', rise_time);
fprintf('Peak Time: %.2f s\n', peak_time);
fprintf('Maximum Peak: %.2f\n', max_peak);
fprintf('Settling Time: %.2f s\n', settling_time);
fprintf('Ess: %.2f\n', ess);

%req8
% Controller parameters
kp = 1; % Proportional gain
% Define the controller transfer function
controller_tf_1 = tf(kp, 1);
closed_loop_tf_1 = feedback(sys2*controller_tf_1, 1);
figure();
 
p_1= stepplot(closed_loop_tf_1);
pzmap (closed_loop_tf_1);
[wn_1,z_1]=damp(closed_loop_tf_1);
tf_1 = closed_loop_tf_1(1);
%check stability
poles_1= pole(tf_1);

% Simulate the system for X
X_1 = lsim(closed_loop_tf_1, Xd, t);



% Plot the responses
figure();
 
plot(t, X_1);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Response of the System to xd=2m, Kp=1');
legend('X_1', 'Xd');

% Calculate the rise time, peak time, maximum peak, settling time, and ess
step_info =stepinfo(closed_loop_tf_1*2);
rise_time = step_info.RiseTime;
peak_time = step_info.PeakTime;
max_peak = step_info.Peak;
settling_time = step_info.SettlingTime;
% Calculate the ess as the difference between the desired signal and the actual signal
ess = Xd(end) - X_1(end);

% Display the results

fprintf('Kp: %.2f \n', kp);
if real(poles_1) < 0
    disp('The system is stable')
else
    disp('The system is unstable')
end
fprintf('Rise Time: %.2f s\n', rise_time);
fprintf('Peak Time: %.2f s\n', peak_time);
fprintf('Maximum Peak: %.2f\n', max_peak);
fprintf('Settling Time: %.2f s\n', settling_time);
fprintf('Ess: %.2f\n', ess);

%kp=10
kp=10;
% Define the controller transfer function
controller_tf_2 = tf(kp, 1);
closed_loop_tf_2 = feedback(sys2*controller_tf_2, 1);
figure();
 
p_2= stepplot(closed_loop_tf_2);
pzmap (closed_loop_tf_2);
[wn_2,z_2]=damp(closed_loop_tf_2);
tf_2 = closed_loop_tf_2(1);
%check stability
poles_2 = pole(tf_2);

% Simulate the system for X
X_2 = lsim(closed_loop_tf_2, Xd, t);

% Plot the responses
figure();
 
plot(t, X_2);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Response of the System to xd=2m, Kp=10');
legend('X_2', 'Xd');

% Calculate the rise time, peak time, maximum peak, settling time, and ess
step_info =stepinfo(closed_loop_tf_2*2);
rise_time = step_info.RiseTime;
peak_time = step_info.PeakTime;
max_peak = step_info.Peak;
settling_time = step_info.SettlingTime;
% Calculate the ess as the difference between the desired signal and the actual signal
ess = Xd(end) - X_2(end);

% Display the results
fprintf('Kp: %.2f \n', kp);
if real(poles_2) < 0
    disp('The system is stable')
else
    disp('The system is unstable')
end

fprintf('Rise Time: %.2f s\n', rise_time);
fprintf('Peak Time: %.2f s\n', peak_time);
fprintf('Maximum Peak: %.2f\n', max_peak);
fprintf('Settling Time: %.2f s\n', settling_time);
fprintf('Ess: %.2f\n', ess);


%kp=100
kp=100;
% Define the controller transfer function
controller_tf_3 = tf(kp, 1);
closed_loop_tf_3 = feedback(sys2*controller_tf_3, 1);
figure();
 
p_3= stepplot(closed_loop_tf_3);
pzmap (closed_loop_tf_3);
[wn_3 ,z_3]=damp(closed_loop_tf_3);
tf_3 = closed_loop_tf_3(1);
%check stability
poles_3 = pole(tf_3);

% Simulate the system for X
X_3= lsim(closed_loop_tf_3, Xd, t);

% Plot the responses
figure();
 
plot(t, X_3);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Response of the System to xd=2m, Kp=100');
legend('X_3', 'Xd');

% Calculate the rise time, peak time, maximum peak, settling time, and ess
step_info =stepinfo(closed_loop_tf_3*2);
rise_time = step_info.RiseTime;
peak_time = step_info.PeakTime;
max_peak = step_info.Peak;
settling_time = step_info.SettlingTime;
% Calculate the ess as the difference between the desired signal and the actual signal
ess = Xd(end) - X_3(end);

% Display the results
fprintf('Kp: %.2f \n', kp);
if real(poles_3) < 0
    disp('The system is stable')
else
    disp('The system is unstable')
end

fprintf('Rise Time: %.2f s\n', rise_time);
fprintf('Peak Time: %.2f s\n', peak_time);
fprintf('Maximum Peak: %.2f\n', max_peak);
fprintf('Settling Time: %.2f s\n', settling_time);
fprintf('Ess: %.2f\n', ess);


%kp=1000
kp=1000;
% Define the controller transfer function
controller_tf_4 = tf(kp, 1);
closed_loop_tf_4 = feedback(sys2*controller_tf_4, 1);
figure();
 
p_4= stepplot(closed_loop_tf_4);
pzmap (closed_loop_tf_4);
[wn_4,z_4]=damp(closed_loop_tf_4);
tf_4 = closed_loop_tf_4(1);
%check stability
poles_4 = pole(tf_4);

% Simulate the system for X
X_4 = lsim(closed_loop_tf_4, Xd, t);

% Plot the responses
figure();
 
plot(t, X_4);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Response of the System to xd=2m, Kp=1000');
legend('X_4', 'Xd');


% Calculate the rise time, peak time, maximum peak, settling time, and ess
step_info =stepinfo(closed_loop_tf_4*2);
rise_time = step_info.RiseTime;
peak_time = step_info.PeakTime;
max_peak = step_info.Peak;
settling_time = step_info.SettlingTime;
% Calculate the ess as the difference between the desired signal and the actual signal
ess = Xd(end) - X_4(end);

% Display the results
fprintf('Kp: %.2f \n', kp);

if real(poles_4) < 0
    disp('The system is stable')
else
    disp('The system is unstable')
end

fprintf('Rise Time: %.2f s\n', rise_time);
fprintf('Peak Time: %.2f s\n', peak_time);
fprintf('Maximum Peak: %.2f\n', max_peak);
fprintf('Settling Time: %.2f s\n', settling_time);
fprintf('Ess: %.2f\n', ess);


%req9
% Simulate the system for X1
Xd_2 = ones(size(t))*4; % Desired displacement
kp = 146;
% Define the controller transfer function
controller_tf_5 = tf(kp, 1);
closed_loop_tf_5 = feedback(sys2*controller_tf_5, 1);
figure();
 
p_5= stepplot(closed_loop_tf_5);
pzmap (closed_loop_tf_5);
[wn_5,z_5]=damp(closed_loop_tf_5);
tf_5 = closed_loop_tf_5(1);
%check stability
poles_5= pole(tf_5);
if real(poles_5) < 0
    disp('The system is stable')
else
    disp('The system is unstable')
end
% Simulate the system for X
X_5 = lsim(closed_loop_tf_5, Xd_2, t);


% Plot the responses
figure();
 
plot(t, X_5);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Response of the System to xd=4m, Kp=400');
legend('X_5', 'Xd');

%req10
ki = 4; % Integral gain
kp=100;
%PI controller
controller_tf_6 = tf([kp ki], [1 0]);

% Connect the controller to the system
sys_with_controller_6 = series(controller_tf_6, sys2);

% Define the closed-loop transfer function
closed_loop_tf_6 = feedback(sys_with_controller_6, 1);
%check stability
figure();
 
p_6= stepplot(closed_loop_tf_6);
pzmap (closed_loop_tf_6);
[wn_6,z_6]=damp(closed_loop_tf_6);
tf_6 = closed_loop_tf_6(1);
%check stability
poles_6= pole(tf_6);
if real(poles_6) < 0
    disp('The system is stable')
else
    disp('The system is unstable')
end
% Simulate the system for X1
X_6 = lsim(closed_loop_tf_6, Xd_2, t);
%calculate ess
ess_system = Xd_2(end) - X_6(end);
fprintf('Ess: %.2f\n', ess_system);
% Plot the responses
figure();
 
plot(t, X_6);
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Response of the System to xd=4m, Kp=100,ki=10');
legend('X_5', 'Xd');