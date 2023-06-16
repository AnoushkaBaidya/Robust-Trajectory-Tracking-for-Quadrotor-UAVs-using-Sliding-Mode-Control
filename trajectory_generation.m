% RBE 502 Final Project 
% Trajectory generation for quadrotor

clear;clc;

% Declaring Variables
syms t t0 tf ti1 tf1 'real'
syms x0 x1 x2 x3 x4 x5 y0 y1 y2 y3 y4 y5 z0 z1 z2 z3 z4 z5 'real'
syms q0 q0_dot q0_ddot qf qf_dot qf_ddot 'real'

% Declaring Position Trajectories 
x = x0 + x1*t + x2*t^2 + x3*t^3 + x4*t^4 + x5*t^5;
y = y0 + y1*t + y2*t^2 + y3*t^3 + y4*t^4 + y5*t^5;
z = z0 + z1*t + z2*t^2 + z3*t^3 + z4*t^4 + z5*t^5;

% Declaring Velocity Trajectories 
x_dot = diff(x, t);
y_dot = diff(y,t);
z_dot = diff(z,t);

% Declaring Acceleration Trajectories 
x_ddot = diff(x_dot, t);
y_ddot = diff(y_dot,t);
z_ddot = diff(z_dot,t);

% Combining into a single vector
q = [x,y,z];
display(q);

% Initialise time variables 
time = [0,5,20,35,50,65];

% Waypoints Initialisation for Trajectory 
p0 = [0,0,0];
p1 = [0,0,1];
p2 = [1,0,1];
p3 = [1,1,1];
p4 = [0,1,1];
p5 = [0,0,1];

waypoints = [p0;p1;p2;p3;p4;p5];

% Initial and Final variable Declaration
q_var = [q0;q0_dot;q0_ddot; qf;qf_dot;qf_ddot];

% Declare matrix to store Coefficient Variables storing 
%[x0 x1 x2 x3 x4 x5;y0 y1 y2 y3 y4 y5;z0 z1 z2 z3 z4 z5];

%Time Matrix 
T = [1, t0, t0^2, t0^3, t0^4, t0^5;...
      0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;...
      0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;...
      1, tf, tf^2, tf^3, tf^4, tf^5;...
      0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;...
      0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];

%Coefficient Matrix generation 
A = inv(T)*q_var;

% Store all the coefficients of trajectories - 5X6X3 
% Store x, y and z Coeffcients 
 x_traj_coeff = [];
 y_traj_coeff  = [];
 z_traj_coeff  = [];

 for i = 1:5
     j = 1;
     coff = subs(A , [t0,tf,q0,qf,q0_dot,q0_ddot,qf_dot,qf_ddot] ,[time(i),time(i+1),waypoints(i,j),waypoints(i+1,j),0,0,0,0]);
     coff = double(coff);
     % Append the coefficients to x_traj
     x_traj_coeff  = horzcat(x_traj_coeff , coff);

 end 

 for i = 1:5
     j = 2;
     coff = subs(A , [t0,tf,q0,qf,q0_dot,q0_ddot,qf_dot,qf_ddot] ,[time(i),time(i+1),waypoints(i,j),waypoints(i+1,j),0,0,0,0]);
     coff = double(coff);
     % Append the coefficients to x_traj
     y_traj_coeff  = horzcat(y_traj_coeff , coff);

 end 

 for i = 1:5
     j = 3;
     coff = subs(A , [t0,tf,q0,qf,q0_dot,q0_ddot,qf_dot,qf_ddot] ,[time(i),time(i+1),waypoints(i,j),waypoints(i+1,j),0,0,0,0]);
     coff = double(coff);
     % Append the coefficients to x_traj
     z_traj_coeff  = horzcat(z_traj_coeff , coff);

  end 

 display(x_traj_coeff);
 display(y_traj_coeff);
 display(z_traj_coeff);

 %Append the Trajectories into Q, Q_dot and Q_ddot 
 Q = sym('Q',[5,3]);
 Q_dot = sym('Q_dot',[5,3]);
 Q_ddot = sym('Q_ddot',[5,3]);

 
for i = 1:5
    j = 1;
     Q(i,j) = subs(q(j),[x0,x1,x2,x3,x4,x5],x_traj_coeff(:,i)');
     

end

for i = 1:5
    j = 2;
    Q(i,j) = subs(q(j),[y0,y1,y2,y3,y4,y5],y_traj_coeff(:,i)');
    

end

for i = 1:5
    j = 3;
     Q(i,j) = subs(q(j),[z0,z1,z2,z3,z4,z5],z_traj_coeff(:,i)');
     

end

for j = 1:3
    for i = 1:5
        if Q(i,j) == 0
            Q_dot(i,j) = 0;
        else
            Q_dot(i,j) = diff(Q(i,j),t);
        end
        if Q_dot(i,j) == 0
            Q_ddot(i,j) = 0;
        else
            Q_ddot(i,j) = diff(Q_dot(i,j),t);
        end
    end
end

% Ts- Time variable from 0 to 65 seconds
Ts =[];
% Variables Declaration to append Values from 0 to 65 seconds
x_traj = [];
xdot_traj = [];
xddot_traj = [];
y_traj = [];
ydot_traj = [];
yddot_traj = [];
z_traj = [];
zdot_traj = [];
zddot_traj = [];

for i = 1:5
    Ts = [Ts , time(i):0.1:time(i+1)];
    x_traj = [x_traj ,subs(Q(i,1),t,time(i):0.1:time(i+1))];
    xdot_traj = [xdot_traj ,subs(Q_dot(i,1),t,time(i):0.1:time(i+1))];
    xddot_traj = [xddot_traj ,subs(Q_ddot(i,1),t,time(i):0.1:time(i+1))];
    
    y_traj = [y_traj ,subs(Q(i,2),t,time(i):0.1:time(i+1))];
    ydot_traj = [ydot_traj ,subs(Q_dot(i,2),t,time(i):0.1:time(i+1))];
    yddot_traj = [yddot_traj ,subs(Q_ddot(i,2),t,time(i):0.1:time(i+1))];
    
    z_traj = [z_traj ,subs(Q(i,3),t,time(i):0.1:time(i+1))];
    zdot_traj = [zdot_traj ,subs(Q_dot(i,3),t,time(i):0.1:time(i+1))];
    zddot_traj = [zddot_traj ,subs(Q_ddot(i,3),t,time(i):0.1:time(i+1))];
end

% Define x_limits
x_limits = [0,65];

% Define coordinate directions and corresponding trajectories
coords = ["x", "y", "z"];
trajs = {x_traj, y_traj, z_traj};
vels = {xdot_traj, ydot_traj, zdot_traj};
accs = {xddot_traj, yddot_traj, zddot_traj};

% Loop over coordinate directions and create figures
for c = 1:3
    % Create figure
    figure('Name',sprintf('Desired Trajectories for Translational Coordinate %s',coords(c)))
    
    % Loop over trajectory components
    for t = 1:3
        % Create subplot
        subplot(3,1,t)
        
        % Plot trajectory component
        switch t
            case 1
                plot(Ts,trajs{c},'linewidth',2 )
                ylabel(sprintf('%s(t)',coords(c)))
            case 2
                plot(Ts,vels{c},'linewidth',2)
                ylabel(sprintf('%s_dot(t)',coords(c)))
            case 3
                plot(Ts,accs{c},'linewidth',2)
                ylabel(sprintf('%s_ddot(t)',coords(c)))
        end
        
        % Add titles and x-limits
        title(sprintf('Desired %s Trajectory',upper(char([coords(c)]))))
        xlabel('time')
        xlim(x_limits)
    end
end


