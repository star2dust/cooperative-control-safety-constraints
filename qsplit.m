function [de,hb,th] = qsplit(q)
% QSPLIT Split q to [de hb th]
de = q(:,1:2,:); hb = q(:,3:5,:);
th = q(:,6:end,:); 

end