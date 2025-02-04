E_1 = 114*10^6; %Pa
E_2 = 7.158*10^6; %Pa
G_12 = 5.847*10^6; %Pa
v_12 = 0.33;

%% PLY MECHANICS

R = [1 0 0; 0 1 0; 0 0 2];
S = [1/E_1 -v_12/E_1 0; -v_12/E_1 1/E_2 0; 0 0 1/G_12];
Q = (S^-1);

%%==============CHANGE LAYUP ANGLES HERE=================
P_ANGLES = [A B C D];

%%

P_TRANS = [cosd(P_ANGLES); sind(P_ANGLES)];

T = zeros(3,3,length(P_ANGLES));
J = zeros(3,3,length(P_ANGLES));
Q_bar = zeros(3,3,length(P_ANGLES));
for i = 1:length(P_ANGLES)
    T(:,:,i) = [P_TRANS(1,i)^2 P_TRANS(2,i)^2 -2*P_TRANS(1,i)*P_TRANS(2,i); P_TRANS(2,i)^2 P_TRANS(1,i)^2 2*P_TRANS(1,i)*P_TRANS(2,i); P_TRANS(1,i)*P_TRANS(2,i) -P_TRANS(1,i)*P_TRANS(2,i) P_TRANS(1,i)^2-P_TRANS(2,i)^2];
    Q_bar(:,:,i) = inv(T(:,:,i))*Q*R*T(:,:,i)*inv(R);
end
    
%% MACROMECHANICS

lamina_thickness = Adherend_Thickness/length(P_ANGLES);
for i = 1:(1+length(P_ANGLES))
z_values(i) = -(Adherend_Thickness/2)+lamina_thickness*(i-1);
end

% A MATRIX
for i = 1:length(P_ANGLES)
Atemp(:,:,i) = Q_bar(:,:,i)*(z_values(i+1)-z_values(i));
end
A = sum(Atemp,3);

%B MATRIX
for i = 1:length(P_ANGLES)
Btemp(:,:,i) = -(1/2)*Q_bar(:,:,i)*((z_values(i+1))^2-(z_values(i))^2);
end
B = sum(Btemp,3);

%D MATRIX
for i = 1:length(P_ANGLES)
Dtemp(:,:,i) = (1/3)*Q_bar(:,:,i)*((z_values(i+1))^3-(z_values(i))^3);
end
D = sum(Dtemp,3);

% ABD MATRIX
ABD = [A B; B D];
INV_ABD = inv(ABD);

% stress-strain calculation
stresses = [10; 0; 0; 0; 0; 0];
Strains = INV_ABD*stresses;
