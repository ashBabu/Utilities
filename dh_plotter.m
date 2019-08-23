clear
close all
temp = eye(4);
nDoF = 3;
l1 = 0.5; l2 = 1.0; l3 = 0.5;  % l3 is the distance towardss the end-effctor
alpha = [-pi/2, pi/2, 0];
a = [0, 0, l2 ];
d = [l1, 0, 0];
q = [pi/2, pi/2, pi/2];
T_joint = zeros(4, 4, nDoF +1);
for i=1:nDoF
    Tx = makehgtform('translate',[a(i) 0 0]);
    Tz = makehgtform('translate',[0 0 d(i)]);
    Rx = makehgtform('xrotate',alpha(i));
    Rz = makehgtform('zrotate', q(i)) ;
    temp = temp * (Rx * Tx * Rz * Tz);
    T_joint(:, :, i) = temp;
end
tmp= eye(4);
tmp(1,4) = l3;
T_joint(:, :, i+1) = temp * tmp;

% plot
x = 0; y = 0; z = 0;

for i=1:length(T_joint)
   plot3(x, y, z, 'k*')
   grid on
   hold on
   plot3([x, T_joint(1, 4, i)], [y, T_joint(2, 4, i)], [z, T_joint(3, 4, i)],'r','LineWidth',2.5)
   x = T_joint(1, 4, i); y = T_joint(2, 4, i); z = T_joint(3, 4, i);
end
xlabel('X')
ylabel('Y')
zlabel('Z')
% axis square

