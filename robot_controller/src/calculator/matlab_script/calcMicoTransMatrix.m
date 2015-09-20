%   ここに行列書いてくれている
%   https://github.com/Kinovarobotics/kinova-ros/blob/master/jaco_driver/src/jaco_arm_kinematics.cpp

syms th1 th2 th3 th4 th5 th6 real
syms xb yb thb lb hb real
% syms d0 d1 d2 d3 d3_offset d4 d5 d6  
syms v w real
syms Rw T positive 

d0 = 0.1544;
d1 = -0.1181;
d2 = 0.2900;
d3_offset = -0.0070;
d3 = 0.1233;
d4 =0.0741;
d5 = 0.0741;
d6 = 0-.1600;
j5_bend =  degtorad(-55);
j6_bend = degtorad(55);

TREE = eye(4);
% origin to arm_base
Trans(:, :, 1) = [ cos(thb), -sin(thb), 0, xb + lb*cos(thb);
    sin(thb),  cos(thb), 0, yb + lb*sin(thb);
    0,         0, 1,                hb;
    0,         0, 0,                1];

%arm_base to j1
Trans(:, :, 1) = Trans(:, :, 1) * [ cos(th1), -sin(th1), 0, 0;
    -sin(th1),  -cos(th1), 0,       0;
    0,         0, -1,     d0;
    0,         0, 0,       1];

%j1 to j2
Trans(:, :, 2) =[ sin(th2),  cos(th2), 0, 0;
    0,         0, 1, 0;
    cos(th2), -sin(th2), 0, d1;
    0,         0, 0, 1];

% j2 to j3
Trans(:, :, 3)=[ -cos(th3), sin(th3), 0, d2;
    sin(th3),  cos(th3), 0,  0;
    0,         0, -1,  0;
    0,         0, 0,  1];

%j3 to j3.5
Trans(:, :, 4)= [ 1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, d3_offset;
    0, 0, 0, 1];

% j3.5 to j4
Trans(:, :, 4) = Trans(:, :, 4) * [0, 0, -1, d3
    sin(th4), cos(th4), 0, 0
    cos(th4), -sin(th4), 0, 0;
    0, 0, 0, 1];

%j4 to j5
Trans(:, :, 5)= [ cos(j5_bend)*cos(th5), cos(j5_bend)*-sin(th5), sin(j5_bend), cos(-j5_bend)*d4;
    sin(th5), cos(th5), 0, 0;
    -sin(j5_bend)*cos(th5), sin(j5_bend)*sin(th5), cos(j5_bend), -sin(-j5_bend)*d4;
    0, 0, 0, 1];

%j5 to j6
Trans(:, :, 6)= [cos(j6_bend) * cos(th6), cos(j6_bend) * -sin(th6), sin(j6_bend), -cos(j6_bend)*d5;
    sin(th6), cos(th6), 0, 0;
    -sin(j6_bend) * cos(th6), sin(j6_bend) * sin(th6), cos(j6_bend), -sin(j6_bend) * d5;
    0, 0, 0, 1];

% j6 to ee
Trans(:, :, 7)= [ -1, 0, 0, 0;
    0, 1, 0,  0;
    0, 0, -1,  d6;
    0, 0, 0,  1];


%% グローバル座標系から手先座標系までの変換行列
for i=1:size(Trans, 3)
    TREE = TREE*Trans(:, :, i);
end

xREE = simplify( TREE(1,4) )
yREE = simplify( TREE(2,4) )
zREE = simplify( TREE(3,4) )
rollREE  = simplify( atan(TREE(3,2)/TREE(3,3)) )
pitchREE = simplify( atan( -TREE(3,1)/sqrt(TREE(3,2)^2 + TREE(3,3)^2) )) 
yawREE   = simplify( atan(  TREE(2,1)/TREE(1,1) ))


%% ヤコビ行列の計算

eeState = [xREE; yREE; zREE; rollREE; pitchREE; yawREE];

% J_armの導出
armState = [th1; th2; th3; th4; th5; th6];
Jarm = jacobian(eeState , armState);

% J_baseの導出
baseState = [xb ; yb; thb];
Jbase = jacobian(eeState , baseState);

% Jの導出
Tnonholo = [ cos(thb) , 0; sin(thb) , 0; 0, 1];
Tconvert = [ Rw/2 Rw/2; Rw/T -Rw/T];
Jbase_nonholo = Jbase*Tnonholo*Tconvert;  

robotState = [armState; baseState];
J = simplify([Jarm , Jbase_nonholo]);

fileID = fopen('./+Results/J.txt','a');
for i = 1:size(J, 1)
    for j = 1:size(J,2)
        fprintf(fileID, 'J[%i, %i] = %s\n\n', i, j, char(J(i,j)));
    end    
end
fprintf(fileID, '\nend');
fclose(fileID);

fileID = fopen('./+Results/DJDq.txt','a');
DJDq = sym(zeros(size(J, 1), size(J, 2), size(robotState, 1)));
for i = 1:size(DJDq, 1)
    for l = 1:size(DJDq, 2)
        for k = 1:size(DJDq, 3)
            DJDq(i, l, k) = diff(J(i, l), robotState(k));
            fprintf(fileID, 'DJDq[%i, %i, %i] = %s\n\n', i-1, j-1, k-1, char(DJDq(i,j,k)));
        end
    end    
end
fprintf(fileID, '\nend');
fclose(fileID);

fileID = fopen('./+Results/DJsqDq.txt','a');
Jsquare = J*J';
DJsqDq = sym(zeros(size(Jsquare, 1), size(Jsquare, 2), size(robotState, 1)));
for i = 1:size(DJsqDq, 1)
    for j = 1:size(DJsqDq, 2)
        for k = 1:size(DJsqDq, 3)
            DJsqDq(i, j, k) = diff(Jsquare(i, j), robotState(k));
            fprintf(fileID, 'DJsqDq[%i, %i, %i] = %s\n\n', i-1, j-1, k-1, char(DJsqDq(i,j,k)));
        end
    end    
end
fprintf(fileID, '\nend');
fclose(fileID);

fileID = fopen('./+Results/DJarmDq.txt','a');
DJarmDq = sym(zeros(size(Jarm, 1), size(Jarm, 2), size(robotState, 1)));
for i = 1:size(DJarmDq, 1)
    for j = 1:size(DJarmDq, 2)
        for k = 1:size(DJarmDq, 3)
            DJarmDq(i, j, k) = diff(Jarm(i, j), robotState(k));
            fprintf(fileID, 'DJarmDq[%i, %i, %i] = %s\n\n', i-1, j-1, k-1, char(DJarmDq(i,j,k)));
        end
    end    
end
fprintf(fileID, '\nend');
fclose(fileID);









