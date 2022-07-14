function fig = robot_animate(fig,robot,th,pe,wr)

%% input data
nrob = length(robot);

%% robot animate
for i=1:nrob
    pbe = PlanarRevolute.getFkine(robot(i).link,th(i,:));
    pb(i,:) = pe(i,:)-pbe;
    robot(i).animate(th(i,:),q(SE2(pb(i,:))),fig.hrob(i).group);
end

%% centroid trajectory
pc = sum(pe)/nrob;
set(fig.hpc,'XData',pc(:,1),'YData',pc(:,2));

%% formation animate
% object shape
ipe = convhull_(pe(:,1:2));
set(fig.hpe,'XData',pe(ipe,1),'YData',pe(ipe,2));

% formation shape 
ipb = convhull_(pb(:,1:2));
set(fig.hpb,'XData',pb(ipb,1),'YData',pb(ipb,2));

%% working range plot
if nargin==5
    for i=1:nrob
        set(fig.hwr(i),'Faces',1:size(wr,1),'Vertices',wr(:,:,i));
    end
end


end