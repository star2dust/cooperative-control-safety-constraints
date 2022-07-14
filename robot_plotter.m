function fig = robot_plotter(robot,th,pe,pr,wr)

%% user data
rob_lk_thick = 2;
rob_hg_size = 2;
rob_ws = pr.ws;
nrob = length(robot);

%% robot plot
for i=1:nrob
    pbe = PlanarRevolute.getFkine(robot(i).link,th(i,:));
    pb(i,:) = pe(i,:)-pbe;
    hrob(i) = robot(i).plot(th(i,:),q(SE2(pb(i,:))),'workspace',rob_ws,...
        'dim',length(rob_ws)/2,'plat','hgsize',rob_hg_size,...
        'lkthick', rob_lk_thick); hold on
end

%% reference trajectory
hpr = plot(pr.p(:,1),pr.p(:,2),'k','LineWidth',1);

%% centroid trajectory
pc = sum(pe)/nrob;
hpc = plot(pc(:,1),pc(:,2),'m*','LineWidth',1);

%% formation plot
% object shape
ipe = convhull_(pe(:,1:2));
hpe = patch(pe(ipe,1),pe(ipe,2),[0.7, 0.7, 0.7],'EdgeColor',[0.5,0.5,0.5],'LineWidth',2); 
% hpe = plot(pe(ipe,1),pe(ipe,2),'Color',[0.5,0.5,0.5],'LineWidth',2);

% formation shape 
ipb = convhull_(pb(:,1:2));
hpb = plot(pb(ipb,1),pb(ipb,2),'r--','LineWidth',2);
    
%% working range plot
if nargin==5
    for i=1:nrob
        hwr(i) = patch('Faces',1:size(wr,1),'Vertices',wr(:,:,i),...
            'LineStyle','none','FaceColor','g','FaceAlpha',0.3);
    end
else
    hwr = [];
end
hold off

%% save data
fig.hrob = hrob;
fig.hpr = hpr;
fig.hpc = hpc;
fig.hpe = hpe;
fig.hpb = hpb;
fig.hwr = hwr;
end