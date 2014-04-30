function genPrimitives()

addpath('./dubins/')

global params;
load_sim_params;

% params.goal = [5;10;pi/2];
% p = dubins([0;0;0]',params.goal', 3, .5);
% nsteps = ceil(1.5*sum(sum(diff(p(1:2,:),[],2).^2))/(params.r_radius*params.d_theta_nom));
% 
% params.numsteps = nsteps;
% 
% 
% A = [eye(params.numsteps); -eye(params.numsteps)];
% b = [ones(params.numsteps,1);ones(params.numsteps,1)];
% u0 = zeros(params.numsteps,1);
% % u = fmincon(@fn,u0,A,b);%,[],[],[],[],@nlcon);
% options = optimset('MaxFunEvals',1e5,'TolCon',1e-6,'MaxIter',1e3);
% u = fmincon(@fn,u0,A,b,[],[],[],[],@nlcon,options);

% u2 = u(u>=-1);
u2 = [ones(42,1);zeros(5,1);-1*ones(42,1)];
x = [0;0;0]
for ii=1:size(u2,1)
    x(:,ii+1) = simDyn(x(:,ii),u2(ii));
end
x(:,end)
hold on
plot(x(1,:),x(2,:),'g')

end


function f = fn(u)
global params;


state = [0;0;0];

for ii=1:params.numsteps
    state = simDyn(state,u(ii));
end


f = norm(u(u>=-1)) + 1e2*norm(state-params.goal) - 1e6*sum(u<-1);
end


function state = simDyn(state,u)
global params;

x = state(1);
y = state(2);
theta = state(3);

% if u >= -1
    r_dTheta = params.d_theta_nom + params.d_theta_max_dev*u;
    l_dTheta = params.d_theta_nom - params.d_theta_max_dev*u;

    R = params.r_radius*r_dTheta;    % distance Right wheel traveled
    L = params.l_radius*l_dTheta;    % distance Left wheel traveled

    if (R == L)
        % Car moved straight
        x = x + (R+L)/2*cos(theta);
        y = y + (R+L)/2*sin(theta);
    else
        % Car moved along an arc
        x = x + params.wb/2*(R+L)/(R-L)*(sin((R-L)/params.wb + theta) - sin(theta));
        y = y - params.wb/2*(R+L)/(R-L)*(cos((R-L)/params.wb + theta) - cos(theta));
    end

    theta = wrapToPi(theta + (R-L)/params.wb);
    
    state = [x;y;theta];
% end


end


function [c ceq] = nlcon(u)

global params;
state = [0;0;0];

for ii=1:params.numsteps
    state = simDyn(state,u(ii));
end

ceq = norm(state-params.goal);
c = [];

end

