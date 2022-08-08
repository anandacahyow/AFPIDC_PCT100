%% ================ Perancangan AFPIDC ================
% PERANCANGAN FUZZY INFERENCE SYSTEM: MF, RULE BASE, DEFFUZ

% Inisialisasi output fuzzy
e=1;
de=1;

% ==== Parameter Membership Function ====
% Proportional Gain
a1 = 1;
b1 = 0.0455;
c1 = 0.432;

% Integral Gain
a2 = 1;
b2 = 0.89875;
c2 = 8.675;

% Derivative Gain
a3 = 1;
b3 = 0.001;
c3 = 0;

% OUTPUT FUZZY
[y1,E_kp,dE_kp,U_kp,fis_kp] = FLC_kp(e,de,a1,b1,c1);
[y2,E_ki,dE_ki,U_ki,fis_ki] = FLC_ki(e,de,a2,b2,c2);
[y3,E_kd,dE_kd,U_kd,fis_kd] = FLC_kd(e,de,a3,b3,c3);

%% ================ Time Domain Spec ================
% WARNING!!! KALAU ERROR: 
% RUN Simulink di dulu, kemudian RUN SECTION
disp('==== Time Domain Spec. Plant with PID ====')
stepinfo(out.PID.Time,out.PID.Data(:,1))

disp('==== Time Domain Spec. Plant with Fuzzy Adaptive ====')
stepinfo(out.AFPIDC.Time,out.AFPIDC.Data(:,1))

% ARX model vs real data
y_simul = out.AFPIDC.Data(:,1);
figure(8)
plot(tt,y)
hold on
plot(tt,y_est)
plot(tt,y_pid)
plot(tt,y_simul)
legend('PCT - 100 flow','Estimated ARX','PID PCT100','AFPIDC')
grid on
title('Flow Measured Value of Experimantal Data vs ARX Model')
xlabel('time(s)')
ylabel('flow(L/min)')
%% ================ ALL FUNCTION =====================

function [y,E,dE,U,fis] = FLC_kp(e,de,a,b,c)
fuzzy_kp = mamfis('Name',"fuzzy_kp");
fuzzy_kp.DefuzzificationMethod="mom";  
fuzzy_kp = addInput(fuzzy_kp,[-4*a 4*a],'Name',"ERROR"); %Adding input 1  
fuzzy_kp = addInput(fuzzy_kp, [-4*a 4*a],'Name',"DERROR"); %Adding input 2
fuzzy_kp = addOutput(fuzzy_kp,[-4*b+c 4*b+c],'Name',"kp"); %vp = control signal pi from the FLC

%membership function of input error
fuzzy_kp = addMF(fuzzy_kp,"ERROR","trimf",[-4*a -3*a -2*a],'Name',"NB"); %Negative Big = 1
fuzzy_kp = addMF(fuzzy_kp,"ERROR","trimf",[-3*a -2*a -a],'Name',"NM"); %Negative Small =2
fuzzy_kp = addMF(fuzzy_kp,"ERROR","trimf",[-2*a -a 0],'Name',"NS"); %Negative Small =3
fuzzy_kp = addMF(fuzzy_kp,"ERROR","trimf", [-a 0 a],'Name',"ZO"); %Zero =4
fuzzy_kp = addMF(fuzzy_kp,"ERROR","trimf", [0 a 2*a],'Name',"PS"); %Positive Small =5
fuzzy_kp = addMF(fuzzy_kp,"ERROR","trimf", [a 2*a 3*a],'Name',"PM"); %Positif Medium =6
fuzzy_kp = addMF(fuzzy_kp,"ERROR","trimf", [2*a 3*a 4*a],'Name',"PB"); %Positive Large =7

%membership function of input derror
fuzzy_kp = addMF(fuzzy_kp,"DERROR","trimf",[-4*a -3*a -2*a],'Name',"NB"); %Negative Big = 1
fuzzy_kp = addMF(fuzzy_kp,"DERROR","trimf",[-3*a -2*a -a],'Name',"NM"); %Negative Small =2
fuzzy_kp = addMF(fuzzy_kp,"DERROR","trimf",[-2*a -a 0],'Name',"NS"); %Negative Small =3
fuzzy_kp = addMF(fuzzy_kp,"DERROR","trimf", [-a 0 a],'Name',"ZO"); %Zero =4
fuzzy_kp = addMF(fuzzy_kp,"DERROR","trimf", [0 a 2*a],'Name',"PS"); %Positive Small =5
fuzzy_kp = addMF(fuzzy_kp,"DERROR","trimf", [a 2*a 3*a],'Name',"PM"); %Positif Medium =6
fuzzy_kp = addMF(fuzzy_kp,"DERROR","trimf", [2*a 3*a 4*a],'Name',"PB"); %Positive Large =7

%membership function of output pi
fuzzy_kp = addMF(fuzzy_kp,"kp","trimf",[-4*b+c -3*b+c -2*b+c],'Name',"NB"); %Negative Big = 1
fuzzy_kp = addMF(fuzzy_kp,"kp","trimf",[-3*b+c -2*b+c -b+c],'Name',"NM"); %Negative Small =2
fuzzy_kp = addMF(fuzzy_kp,"kp","trimf",[-2*b+c -b+c 0+c],'Name',"NS"); %Negative Small =3
fuzzy_kp = addMF(fuzzy_kp,"kp","trimf", [-b+c 0+c b+c],'Name',"ZO"); %Zero =4
fuzzy_kp = addMF(fuzzy_kp,"kp","trimf", [0+c b+c 2*b+c],'Name',"PS"); %Positive Small =5
fuzzy_kp = addMF(fuzzy_kp,"kp","trimf", [b+c 2*b+c 3*b+c],'Name',"PM"); %Positif Medium =6
fuzzy_kp = addMF(fuzzy_kp,"kp","trimf", [2*b+c 3*b+c 4*b+c],'Name',"PB"); %Positive Large =7

% Rules List = [[Input] [Second Input] [Output] Weight(Usually 1) [AND/OR operator =>1:AND, 2:OR]]
rules_fuzzy =[...
    1 1 7 1 1;
    1 2 7 1 1;
    1 3 7 1 1;
    1 4 7 1 1;
    1 5 6 1 1;
    1 6 5 1 1;
    1 7 4 1 1;
    2 1 7 1 1;
    2 2 7 1 1;
    2 3 7 1 1;
    2 4 6 1 1;
    2 5 5 1 1;
    2 6 4 1 1;
    2 7 3 1 1;
    3 1 6 1 1;
    3 2 6 1 1;
    3 3 5 1 1;
    3 4 5 1 1;
    3 5 4 1 1;
    3 6 3 1 1;
    3 7 3 1 1;
    4 1 6 1 1;
    4 2 5 1 1;
    4 3 5 1 1;
    4 4 4 1 1;
    4 5 3 1 1;
    4 6 3 1 1;
    4 7 2 1 1;
    5 1 5 1 1;
    5 2 5 1 1;
    5 3 4 1 1;
    5 4 3 1 1;
    5 5 3 1 1;
    5 6 2 1 1;
    5 7 2 1 1;
    6 1 5 1 1;
    6 2 4 1 1;
    6 3 3 1 1;
    6 4 2 1 1;
    6 5 1 1 1;
    6 6 1 1 1;
    6 7 1 1 1;
    7 1 4 1 1;
    7 2 3 1 1;
    7 3 2 1 1;
    7 4 1 1 1;
    7 5 1 1 1;
    7 6 1 1 1;
    7 7 1 1 1];
%adding rules to the fis
fuzzy_kp = addRule(fuzzy_kp,rules_fuzzy);

%plotting the MFs
figure(5)
subplot(2,2,1)
plotmf(fuzzy_kp,'input',1,1000)
title('Error Input Membership Function')
subplot(2,2,2)
plotmf(fuzzy_kp,'input',2,1000)
title('dError Input Membership Function')
subplot(2,2,3)
plotmf(fuzzy_kp,'output',1,1000)
title('Dv Output Membership Function')
subplot(2,2,4)
gensurf(fuzzy_kp)
y = evalfis(fuzzy_kp,[e de]);
[E,dE,U] = gensurf(fuzzy_kp);
fis = writeFIS(fuzzy_kp,"FUZZY_KP")
end

function [y,E,dE,U,fis] = FLC_ki(e,de,a,b,c)
fuzzy_ki = mamfis('Name',"fuzzy_ki");
fuzzy_ki.DefuzzificationMethod="mom";  
fuzzy_ki = addInput(fuzzy_ki,[-4*a 4*a],'Name',"ERROR"); %Adding input 1  
fuzzy_ki = addInput(fuzzy_ki, [-4*a 4*a],'Name',"DERROR"); %Adding input 2
fuzzy_ki = addOutput(fuzzy_ki,[-4*b+c 4*b+c],'Name',"ki"); %vp = control signal pi from the FLC

%membership function of input error
fuzzy_ki = addMF(fuzzy_ki,"ERROR","trimf",[-4*a -3*a -2*a],'Name',"NB"); %Negative Big = 1
fuzzy_ki = addMF(fuzzy_ki,"ERROR","trimf",[-3*a -2*a -a],'Name',"NM"); %Negative Small =2
fuzzy_ki = addMF(fuzzy_ki,"ERROR","trimf",[-2*a -a 0],'Name',"NS"); %Negative Small =3
fuzzy_ki = addMF(fuzzy_ki,"ERROR","trimf", [-a 0 a],'Name',"ZO"); %Zero =4
fuzzy_ki = addMF(fuzzy_ki,"ERROR","trimf", [0 a 2*a],'Name',"PS"); %Positive Small =5
fuzzy_ki = addMF(fuzzy_ki,"ERROR","trimf", [a 2*a 3*a],'Name',"PM"); %Positif Medium =6
fuzzy_ki = addMF(fuzzy_ki,"ERROR","trimf", [2*a 3*a 4*a],'Name',"PB"); %Positive Large =7

%membership function of input derror
fuzzy_ki = addMF(fuzzy_ki,"DERROR","trimf",[-4*a -3*a -2*a],'Name',"NB"); %Negative Big = 1
fuzzy_ki = addMF(fuzzy_ki,"DERROR","trimf",[-3*a -2*a -a],'Name',"NM"); %Negative Small =2
fuzzy_ki = addMF(fuzzy_ki,"DERROR","trimf",[-2*a -a 0],'Name',"NS"); %Negative Small =3
fuzzy_ki = addMF(fuzzy_ki,"DERROR","trimf", [-a 0 a],'Name',"ZO"); %Zero =4
fuzzy_ki = addMF(fuzzy_ki,"DERROR","trimf", [0 a 2*a],'Name',"PS"); %Positive Small =5
fuzzy_ki = addMF(fuzzy_ki,"DERROR","trimf", [a 2*a 3*a],'Name',"PM"); %Positif Medium =6
fuzzy_ki = addMF(fuzzy_ki,"DERROR","trimf", [2*a 3*a 4*a],'Name',"PB"); %Positive Large =7

%membership function of output pi
fuzzy_ki = addMF(fuzzy_ki,"ki","trimf",[-4*b+c -3*b+c -2*b+c],'Name',"NB"); %Negative Big = 1
fuzzy_ki = addMF(fuzzy_ki,"ki","trimf",[-3*b -2*b -b]+c,'Name',"NM"); %Negative Small =2
fuzzy_ki = addMF(fuzzy_ki,"ki","trimf",[-2*b -b 0]+c,'Name',"NS"); %Negative Small =3
fuzzy_ki = addMF(fuzzy_ki,"ki","trimf", [-b 0 b]+c,'Name',"ZO"); %Zero =4
fuzzy_ki = addMF(fuzzy_ki,"ki","trimf", [0 b 2*b]+c,'Name',"PS"); %Positive Small =5
fuzzy_ki = addMF(fuzzy_ki,"ki","trimf", [b 2*b 3*b]+c,'Name',"PM"); %Positif Medium =6
fuzzy_ki = addMF(fuzzy_ki,"ki","trimf", [2*b 3*b 4*b]+c,'Name',"PB"); %Positive Large =7

% Rules List = [[Input] [Second Input] [Output] Weight(Usually 1) [AND/OR operator =>1:AND, 2:OR]]
rules_fuzzy =[...
    1 1 7 1 1;
    1 2 7 1 1;
    1 3 7 1 1;
    1 4 7 1 1;
    1 5 6 1 1;
    1 6 5 1 1;
    1 7 4 1 1;
    2 1 7 1 1;
    2 2 7 1 1;
    2 3 7 1 1;
    2 4 6 1 1;
    2 5 5 1 1;
    2 6 4 1 1;
    2 7 3 1 1;
    3 1 6 1 1;
    3 2 6 1 1;
    3 3 5 1 1;
    3 4 5 1 1;
    3 5 4 1 1;
    3 6 3 1 1;
    3 7 3 1 1;
    4 1 6 1 1;
    4 2 5 1 1;
    4 3 5 1 1;
    4 4 4 1 1;
    4 5 3 1 1;
    4 6 3 1 1;
    4 7 2 1 1;
    5 1 5 1 1;
    5 2 5 1 1;
    5 3 4 1 1;
    5 4 3 1 1;
    5 5 3 1 1;
    5 6 2 1 1;
    5 7 2 1 1;
    6 1 5 1 1;
    6 2 4 1 1;
    6 3 3 1 1;
    6 4 2 1 1;
    6 5 1 1 1;
    6 6 1 1 1;
    6 7 1 1 1;
    7 1 4 1 1;
    7 2 3 1 1;
    7 3 2 1 1;
    7 4 1 1 1;
    7 5 1 1 1;
    7 6 1 1 1;
    7 7 1 1 1];
%adding rules to the fis
fuzzy_ki = addRule(fuzzy_ki,rules_fuzzy);

%plotting the MFs
figure(6)
subplot(2,2,1)
plotmf(fuzzy_ki,'input',1,1000)
title('Error Input Membership Function')
subplot(2,2,2)
plotmf(fuzzy_ki,'input',2,1000)
title('dError Input Membership Function')
subplot(2,2,3)
plotmf(fuzzy_ki,'output',1,1000)
title('Dv Output Membership Function')
subplot(2,2,4)
gensurf(fuzzy_ki)
%test point
y = evalfis(fuzzy_ki,[e de]);
[E,dE,U] = gensurf(fuzzy_ki);
fis = writeFIS(fuzzy_ki,"FUZZY_KI")
end

function [y,E,dE,U,fis] = FLC_kd(e,de,a,b,c)
fuzzy_kd = mamfis('Name',"fuzzy_kd");
fuzzy_kd.DefuzzificationMethod="mom";  
fuzzy_kd = addInput(fuzzy_kd,[-4*a 4*a],'Name',"ERROR"); %Adding input 1  
fuzzy_kd = addInput(fuzzy_kd, [-4*a 4*a],'Name',"DERROR"); %Adding input 2
fuzzy_kd = addOutput(fuzzy_kd,[-4*b+c 4*b+c],'Name',"kd"); %vp = control signal pi from the FLC

%membership function of input error
fuzzy_kd = addMF(fuzzy_kd,"ERROR","trimf",[-4*a -3*a -2*a],'Name',"NB"); %Negative Big = 1
fuzzy_kd = addMF(fuzzy_kd,"ERROR","trimf",[-3*a -2*a -a],'Name',"NM"); %Negative Small =2
fuzzy_kd = addMF(fuzzy_kd,"ERROR","trimf",[-2*a -a 0],'Name',"NS"); %Negative Small =3
fuzzy_kd = addMF(fuzzy_kd,"ERROR","trimf", [-a 0 a],'Name',"ZO"); %Zero =4
fuzzy_kd = addMF(fuzzy_kd,"ERROR","trimf", [0 a 2*a],'Name',"PS"); %Positive Small =5
fuzzy_kd = addMF(fuzzy_kd,"ERROR","trimf", [a 2*a 3*a],'Name',"PM"); %Positif Medium =6
fuzzy_kd = addMF(fuzzy_kd,"ERROR","trimf", [2*a 3*a 4*a],'Name',"PB"); %Positive Large =7

%membership function of input derror
fuzzy_kd = addMF(fuzzy_kd,"DERROR","trimf",[-4*a -3*a -2*a],'Name',"NB"); %Negative Big = 1
fuzzy_kd = addMF(fuzzy_kd,"DERROR","trimf",[-3*a -2*a -a],'Name',"NM"); %Negative Small =2
fuzzy_kd = addMF(fuzzy_kd,"DERROR","trimf",[-2*a -a 0],'Name',"NS"); %Negative Small =3
fuzzy_kd = addMF(fuzzy_kd,"DERROR","trimf", [-a 0 a],'Name',"ZO"); %Zero =4
fuzzy_kd = addMF(fuzzy_kd,"DERROR","trimf", [0 a 2*a],'Name',"PS"); %Positive Small =5
fuzzy_kd = addMF(fuzzy_kd,"DERROR","trimf", [a 2*a 3*a],'Name',"PM"); %Positif Medium =6
fuzzy_kd = addMF(fuzzy_kd,"DERROR","trimf", [2*a 3*a 4*a],'Name',"PB"); %Positive Large =7

%membership function of output pi
fuzzy_kd = addMF(fuzzy_kd,"kd","trimf",[-4*b -3*b -2*b]+c,'Name',"NB"); %Negative Big = 1
fuzzy_kd = addMF(fuzzy_kd,"kd","trimf",[-3*b -2*b -b]+c,'Name',"NM"); %Negative Small =2
fuzzy_kd = addMF(fuzzy_kd,"kd","trimf",[-2*b -b 0]+c,'Name',"NS"); %Negative Small =3
fuzzy_kd = addMF(fuzzy_kd,"kd","trimf", [-b 0 b]+c,'Name',"ZO"); %Zero =4
fuzzy_kd = addMF(fuzzy_kd,"kd","trimf", [0 b 2*b]+c,'Name',"PS"); %Positive Small =5
fuzzy_kd = addMF(fuzzy_kd,"kd","trimf", [b 2*b 3*b]+c,'Name',"PM"); %Positif Medium =6
fuzzy_kd = addMF(fuzzy_kd,"kd","trimf", [2*b 3*b 4*b]+c,'Name',"PB"); %Positive Large =7

% Rules List = [[Input] [Second Input] [Output] Weight(Usually 1) [AND/OR operator =>1:AND, 2:OR]]
rules_fuzzy =[...
    1 1 7 1 1;
    1 2 7 1 1;
    1 3 7 1 1;
    1 4 7 1 1;
    1 5 6 1 1;
    1 6 5 1 1;
    1 7 4 1 1;
    2 1 7 1 1;
    2 2 7 1 1;
    2 3 7 1 1;
    2 4 6 1 1;
    2 5 5 1 1;
    2 6 4 1 1;
    2 7 3 1 1;
    3 1 6 1 1;
    3 2 6 1 1;
    3 3 5 1 1;
    3 4 5 1 1;
    3 5 4 1 1;
    3 6 3 1 1;
    3 7 3 1 1;
    4 1 6 1 1;
    4 2 5 1 1;
    4 3 5 1 1;
    4 4 4 1 1;
    4 5 3 1 1;
    4 6 3 1 1;
    4 7 2 1 1;
    5 1 5 1 1;
    5 2 5 1 1;
    5 3 4 1 1;
    5 4 3 1 1;
    5 5 3 1 1;
    5 6 2 1 1;
    5 7 2 1 1;
    6 1 5 1 1;
    6 2 4 1 1;
    6 3 3 1 1;
    6 4 2 1 1;
    6 5 1 1 1;
    6 6 1 1 1;
    6 7 1 1 1;
    7 1 4 1 1;
    7 2 3 1 1;
    7 3 2 1 1;
    7 4 1 1 1;
    7 5 1 1 1;
    7 6 1 1 1;
    7 7 1 1 1];
%adding rules to the fis
fuzzy_kd = addRule(fuzzy_kd,rules_fuzzy);

%plotting the MFs
figure(7)
subplot(2,2,1)
plotmf(fuzzy_kd,'input',1,1000)
title('Error Input Membership Function')
subplot(2,2,2)
plotmf(fuzzy_kd,'input',2,1000)
title('dError Input Membership Function')
subplot(2,2,3)
plotmf(fuzzy_kd,'output',1,1000)
title('Dv Output Membership Function')
subplot(2,2,4)
gensurf(fuzzy_kd)
%test point
y = evalfis(fuzzy_kd,[e de]);
[E,dE,U] = gensurf(fuzzy_kd);
fis = writeFIS(fuzzy_kd,"FUZZY_KD")
end