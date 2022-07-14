function [dqt,dnu] = robot_optimizer(robot,qt,nu,beta,opt)

%% input data
[de,hb,th] = qsplit(qt);
lam = nu(:,1:3); eta = nu(:,4:end);
nrob = length(robot);
Tb = opt.Tb;
D = opt.D;
Qrob = opt.Q;
th_ref = opt.th_ref;
Db_ref = opt.Db_ref;
simu_env = opt.env;
L = D*D';
%% primal-dual algorithm - primal
% cost function
kappa = 1.5;
Wb = eye(2);
Wa = eye(size(th,2)); 
for i=1:nrob
    pbe(i,:) = PlanarRevolute.getFkine(robot(i).link,th(i,:));
    Jbe = PlanarRevolute.getJacob(robot(i).link,th(i,:));
    % written in column
    uq1(i,:) = [Tb(:,:,i)';-Db_ref(:,:,i)'*Tb(:,:,i)';-Jbe'*Tb(:,:,i)']*...
        Wb*Tb(:,:,i)*(de(i,:)'-Db_ref(:,:,i)*hb(i,:)'-pbe(i,:)');
end
uobs = zeros(nrob,2);
uhb = zeros(size(hb));
if simu_env == 2
    for i=1:nrob
        ralpha = 0.1;
        rsafe = 0.2;
        rdetect = 5;
        for j=1:size(opt.po(:,:,i),1)
            pb_real = hb(i,:)*Db_ref(:,:,i)';
            po_real = opt.po(j,:,i);
            pbo_dis = norm(pb_real-po_real)-opt.ro;
            if pbo_dis<=rdetect
                Qrob=opt.Q_min;
                % unit vector at db
                db_nv = de(i,:)/norm(de(i,:));
                po_near = opt.ro*(pb_real-po_real)/norm(pb_real-po_real);
                % project to db
                uobs(i,:) = uobs(i,:)-(ralpha*log(pbo_dis/rsafe)*(pb_real-po_near)./(pbo_dis.^2)*db_nv')*db_nv;
            else
                Qrob = opt.Q;
            end
        end
        uhb(i,:) = uobs(i,:)*pinv(Db_ref(:,:,i)');
    end
end
uq2 = [kappa*eta,kappa*L*lam+uhb,(th-th_ref)*Wa];
uq = uq1+uq2;
% projection
for i=1:nrob
    quq(i,:)= convproj(qt(i,:)-uq(i,:),Qrob(i).A,Qrob(i).b,Qrob(i).lim);
    dqt(i,:) = quq(i,:)-qt(i,:);
end
% dqrob split
[dde,dhb,dth] = qsplit(dqt);
%% primal-dual algorithm - dual
dlam = kappa*L*(dhb+hb);
deta = kappa*(de+dde-beta*D*sign(D'*eta));
% dlambda
dnu = [dlam,deta];
end