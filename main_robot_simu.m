% robot simulation for formation

simu_debug = true;
if simu_debug
    close all
    clear
    % load data
    load('initial_data');
    simu_debug = true;
end

%% control parameters
alpha = 0.5;
gamma = 2;
beta = norm(de_lim(2,:))*sqrt(N_ROB);

%% initial variables
pcr = pc_ref.p(1,:);
pc_hat = pe0+rand([5,2])-0.5;
pe = pe0;
qt = q0;
[de,hb,th] = qsplit(qt);
nu = zeros(N_ROB,5);

% working range
dis = 10*TILE_SIZ;
for i=1:N_ROB
    pfb = (Db_ref(:,:,i)*hb(i,:)')';
    pf = pe(i,:)-de(i,:);
    pb_ref = pf+pfb;
    wrb = [pb_ref;pb_ref+[cos(thbT_min(i)),sin(thbT_min(i))]*dis;
    pb_ref+[cos(thbT_max(i)),sin(thbT_max(i))]*dis;pb_ref];
    wre = [pe(i,:);pe(i,:)+[cos(theT_min(i)),sin(theT_min(i))]*dis;
        pe(i,:)+[cos(theT_max(i)),sin(theT_max(i))]*dis;pe(i,:)];
    wr(:,:,i) = polyxpoly_(wrb,wre);
end

% zero velocities
dpc_hat = zeros(size(pc_hat));
dpe = zeros(size(pe));
dqt = zeros(size(qt));
dnu = zeros(size(nu));
[dde,dhb,dth] = qsplit(dqt);

%% simulation plot
fig = robot_plotter(robot_object,th,pe,pc_ref,wr);
xlabel('$x$ (m)','Fontsize',20,'Interpreter','latex');
ylabel('$y$ (m)','Fontsize',20,'Interpreter','latex');
set(gcf,'unit','normalized','color',[1,1,1])
if simu_env == 2
    ro = ROB_RADIUS*4;
    po_dis = db_siz*[0.75,0.65]'*TILE_SIZ+ROB_RADIUS+ro;
    po = [10,10;13,13]+po_dis.*[-1,1;1,-1]/sqrt(2);
    hold on
    for i=1:size(po,1)
        Vo = circle_(po(i,:),ro,'n',20);
        fig.hpo(i) = patch('Faces',1:size(Vo,1),'Vertices',Vo,...
            'LineStyle','none','FaceColor','k','FaceAlpha',0.8);
        Vo_safe = circle_(po(i,:),ro+ROB_RADIUS,'n',20);
        fig.hpo_safe(i) = plot(Vo_safe(:,1),Vo_safe(:,2),'r','LineWidth',1);
    end
    hold off
end

%% simulation loop - fixed dt
loop = 0;
playspeed = 4;
dt = 0.02;
video_on = false;
save_on = false;
for t = pc_ref.t(1):dt:pc_ref.t(end)
    loop = loop+1;
    % get pose trajectory
    dpcr = interp1(pc_ref.t,pc_ref.dp,t);
    % robot controller - estimation
    dpc_hat = kron(ones(N_ROB,1),dpcr)...
        -gamma*(pc_hat-kron(ones(N_ROB,1),pcr));
    % robot controller - tracking 
    [sig_pf,sig_done] = sig(D'*(pe-de),alpha); % sign(D'*qf).*abs(D'*qf).^alpha;
    dpe = dde + dpc_hat-gamma*((pe-de)-pc_hat)-D*sig_pf;   
    % robot controller - optimization
    opt.Tb = Tb;
    opt.D = D;
    opt.Q = Q;
    opt.Q_min = Q_min;
    opt.th_ref = th_ref;
    opt.Db_ref = Db_ref;
    opt.env = simu_env;
    if simu_env == 2
        opt.ro = ro+ROB_RADIUS;
        for i=1:N_ROB
            opt.po(:,:,i) = po-(pe(i,:)-de(i,:)); 
        end
    end
    [dqt,dnu] = robot_optimizer(robot_object,qt,nu,beta,opt);
    % update pose
    pcr = pcr + dpcr*dt;
    pc_hat = pc_hat + dpc_hat*dt;
    pe = pe + dpe*dt;
    qt = qt + dqt*dt;
    nu = nu + dnu*dt;
    pc = sum(pe)/N_ROB;
    % qt split
    [de,hb,th] = qsplit(qt);
    [dde,dhb,dth] = qsplit(dqt);
    % working range
    for i=1:N_ROB
        pfb = (Db_ref(:,:,i)*hb(i,:)')';
        pf = pe(i,:)-de(i,:);
        pb_ref = pf+pfb;
        wrb = [pb_ref;pb_ref+[cos(thbT_min(i)),sin(thbT_min(i))]*dis;
            pb_ref+[cos(thbT_max(i)),sin(thbT_max(i))]*dis;pb_ref];
        wre = [pe(i,:);pe(i,:)+[cos(theT_min(i)),sin(theT_min(i))]*dis;
            pe(i,:)+[cos(theT_max(i)),sin(theT_max(i))]*dis;pe(i,:)];
        wr(:,:,i) = polyxpoly_(wrb,wre);
    end
    % save data
    simu.t(loop) = t;
    simu.pc_ref(loop,:) = pcr;
    simu.pc(loop,:) = pc;
    for i=1:N_ROB
        simu.pc_hat(loop,:,i) = pc_hat(i,:);
        simu.qt(loop,:,i) = qt(i,:);
        simu.nu(loop,:,i) = nu(i,:);
        simu.pe(loop,:,i) = pe(i,:);
    end
    % update figure
    if mod(loop,playspeed)==0
        fig = robot_animate(fig,robot_object,th,pe,wr);
        if video_on
            frame(loop/playspeed) = getframe(gcf);
        end
    end
    % next loop
    drawnow
end
% write video
if video_on 
    savevideo('simu_video',frame);
end
% data to mat
if save_on
    save('simu_data');
end