
yalmip('clear')
clearvars -except Xhist2
close all
tic
UTN = UTN_setup();

UTN.Options.Empty_output_links = true;         %Controls if we flush the output links to the larger network
%% Check Function that all traffic lights have assigned turning rates
% for i=1:length(UTN.Traffic_lights)
%     if UTN.Turning_rates(UTN.Traffic_lights{i}(1),UTN.Traffic_lights{i}(2),UTN.Traffic_lights{i}(3)) == 0
%        flag = 1;
%     end
% end

%% Model Implementation
%Model data
nx = length(UTN.Links);        %Number of States
nu = length(UTN.Traffic_lights); %Number of Inputs
A = eye(length(UTN.Links)) - diag(UTN.Parking_rates - UTN.Merging_rates);

% Model data
B = sdpvar(14,6,16);
E = ones(nx);

% MPC data
Q = 20; %eye(nx);
R = 2*eye(nu);
N = 7;

disturbance = 0*UTN.Nominal_inflow;
ext_links = find(UTN.Links(:,1) > 6 | UTN.Links(:,2) > 6);
r = 10*zeros(nx,1) ;
r(ext_links) = 10;
r = 10*ones(nx,1);
xsim = 1*300*ones(18,1); 

Simlength = 50;
Xhist = zeros(length(UTN.Links),Simlength+1);
Xhist(:,1) = xsim; Uhist = zeros(length(UTN.Traffic_lights),Simlength); 
sel_links = find(UTN.Links(:,2)<=6); 
for t = 1:Simlength
    
%disturbance = 0*UTN.Nominal_inflow;
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
slack_var =  sdpvar(repmat(nx,1,N),repmat(1,1,N)); 
constraints = []; 
objective = 0;
%% TODO
% Include the min condition in the dynamics function
% Include constraints so that traffic lights don't "crash"
x{1} = xsim; 
queue = zeros(nu,1);
 for k = 1:N
     if k == floor(N/2)
         disturbance = 0*UTN.Nominal_inflow;
     end
     [x{k+1} queue] = lower_dynamics_expanded(x{k},u{k}, disturbance, queue, k, UTN);
     objective = objective + (x{k+1}(sel_links)-r(sel_links))'*Q*(x{k+1}(sel_links)-r(sel_links)) + 0.0001*u{k}'*u{k};
     constraints = [constraints, 5 <= u{k}<= 120 x{k+1}>=slack_var{k}, slack_var{k}(sel_links)>=0]; 
     for i = UTN.Intersections
         idx = find(UTN.Traffic_lights(:,2) == i);
         constraints = [constraints, sum(u{k}(idx))<= UTN.Cycle(i)];
     end
     objective = objective + 1e5*slack_var{k}'*slack_var{k}; 
 end
 ops = sdpsettings('solver','gurobi'); 
 diagnostics = optimize(constraints, objective, ops); 
 xsim = lower_dynamics_expanded(xsim,value(u{1}), disturbance, queue, 1, UTN);
 Xhist(:,t+1) = xsim; 
 Uhist(:,t) = value(u{1}); 
 
  if strcmp(diagnostics.info, ...
         'Either infeasible or unbounded (<a href="yalmip.github.io/infeasibleorunbounded">learn to debug</a>) (GUROBI-NONCONVEX)') == 1
     fprintf('Simulation terminated after t = %d iterations\n', t)
     Xhist = Xhist(:,1:t+1);
     break;
 end
 
end
toc
int_links = find(UTN.Links(:,1) <= 6 & UTN.Links(:,2) <= 6);

hold on
plot(Xhist(1,:))
plot(Xhist(2,:))
plot(Xhist(3,:))
plot(Xhist(4,:))
plot(Xhist(7,:))
plot(Xhist(9,:))
plot(Xhist(10,:))
plot(Xhist(11,:))
%  clf;
%  oldu = zeros(length(UTN.Traffic_lights),1);
%  hold on
%  xhist = x;
%  for i = 1:3
%       future_r = zeros(18,1);
% %     future_r = 3*sin((i:i+N)/40);    
%       inputs = {x,future_r,disturbance,oldu};
%       [solutions,diagnostics] = controller{inputs};    
% %     U = solutions{1};oldu = U(1);
% %     X = solutions{2};
% %     if diagnostics == 1
% %         error('The problem is infeasible');
% %     end    
% %     subplot(1,2,1);stairs(i:i+length(U)-1,U,'r')
% %     subplot(1,2,2);cla;stairs(i:i+N,X(1,:),'b');hold on;stairs(i:i+N,future_r(1,:),'k')
% %     stairs(1:i,xhist(1,:),'g')    
%       x = lower_dynamics(x,U(1), d, k, UTN);
%       xhist = [xhist x];
% %     pause(0.05)   
% %     % The measured disturbance actually isn't constant, it changes slowly
% %     disturbance = 0.99*disturbance + 0.01*randn(1);
% end