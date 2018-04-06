clear;
flag_x1 = 1;
flag_x2 = 1;
flag_xj = 1;
t=0.05;
% ch = 0;
ch = 0;
%  m = 260;
%  m = 239;
TxtData1 = importdata('cam1.txt');
armjoints = importdata('ralPointFile.txt');
PnpResult1 = importdata('pnp1.txt');
TxtData2 = importdata('cam2.txt');
m = size(TxtData1,1);
% Rt=[0.891645542812330,-0.0253519132701749,-0.452023789724000;0.0468332400881701,0.998239479040235,0.0363949187157362;0.450305311545122,-0.0536211057236119,0.891263094386404];
kx1 = 847.421218293945;ky1 = 842.849732532109;u01 = 319.399607306323;v01 = 264.829901470455;
kx2 = 818.086365732527;ky2 = 817.128180829000;u02 = 361.905958857945;v02 = 226.845448574816;
tt = [112.005196173910;3.16303348852451;-1.69908506874230];
% euler2 = rodrigues(Rt)';
euler2 =  [-0.0572
   -0.23922
    0.0254];
% euler2 =  [0.0254
%    -0.23922
%     -0.0572];
Rt = rodrigues(euler2);
T = [Rt,tt];
Init_X1 = [PnpResult1(1,1);0;PnpResult1(1,2);0;PnpResult1(1,3);0;PnpResult1(1,4)/180*pi;0;PnpResult1(1,5)/180*pi;0;PnpResult1(1,6)/180*pi;0];
x1 = Init_X1;
x2 = Init_X1;x3 = x1;

P1 = eye(12);P2 = eye(12);P3  = P2 ;
focalIndex = [kx1 ky1 u01 v01;kx2 ky2 u02 v02]';
RelatObjCoor = [-35,-80,0;
    35,-80,0; 
    35,-10,0; 
    -35,-10,0;
    -20,-65,0;
    20,-65,0; 
    20,-25,0; 
    -20,-25,0];
N = 30;
qNk1 = zeros(12,N);qNk2 = zeros(12,N);
detNk1 = zeros(N*12,12);detNk2 = zeros(N*12,12);
for i = 1:m
    z1 = TxtData1(i,:)';z2 = TxtData2(i,:)';z=[z1,z2];
%     
    [ x1,P1 ] = NonlinerEKF8(z1,x1,P1,focalIndex,t,RelatObjCoor,euler2,T,10,1);
    [ x2,P2 ] = NonlinerEKF8(z2,x2,P2,focalIndex,t,RelatObjCoor,euler2,T,10,2);
    [ x3,P3 ] = NonlinerEKF8(z,x3,P3,focalIndex,t,RelatObjCoor,euler2,T,10,5);
    SData_X1(i,:) = abs([x1(1),x1(3),x1(5),x1(7)*180/pi,x1(9)*180/pi,x1(11)*180/pi]-armjoints(i,:));
    SData_X2(i,:) = abs([x2(1),x2(3),x2(5),x2(7)*180/pi,x2(9)*180/pi,x2(11)*180/pi]-armjoints(i,:));
    SData_X3(i,:) = abs([x3(1),x3(3),x3(5),x3(7)*180/pi,x3(9)*180/pi,x3(11)*180/pi]-armjoints(i,:));

%     [ x2,P2 ] = NonlinerUKF8(z,x2,P2,focalIndex,t,RelatObjCoor,euler2,T,5);

%     [ x1,P1] = NonlinerEKF8(z1,x1,P1,focalIndex,t,RelatObjCoor,euler2,T,N,1);
%     [ x2,P2] = NonlinerEKF8(z2,x2,P2,focalIndex,t,RelatObjCoor,euler2,T,N,2);
%     [ x3,P3] = NonlinerEKF8(z,x3,P3,focalIndex,t,RelatObjCoor,euler2,T,N,4);
%     SData_X1(i,:) = abs([x1(1),x1(3),x1(5),x1(11)*180/pi,x1(9)*180/pi,x1(7)*180/pi]-armjoints(i,:));
%     SData_X2(i,:) = abs([x2(1),x2(3),x2(5),x2(11)*180/pi,x2(9)*180/pi,x2(7)*180/pi]-armjoints(i,:));
%     SData_X3(i,:) = abs([x3(1),x3(3),x3(5),x3(11)*180/pi,x3(9)*180/pi,x3(7)*180/pi]-armjoints(i,:));
end

% a = 1:m;
% SData_X1(:,7) =SData_X1(:,7)*180/pi;
% SData_X1(:,9) = SData_X1(:,9)*180/pi;
% SData_X1(:,11) = SData_X1(:,11)*180/pi;
% SData_X2(:,7) =SData_X2(:,7)*180/pi;
% SData_X2(:,9) = SData_X2(:,9)*180/pi;
% SData_X2(:,11) = SData_X2(:,11)*180/pi;
% SData_X3(:,7) =SData_X3(:,7)*180/pi;
% SData_X3(:,9) = SData_X3(:,9)*180/pi;
% SData_X3(:,11) = SData_X3(:,11)*180/pi;
% figure(4);
% 
% if flag_x1 
%     subplot(3,2,1);
%     plot(a,SData_X1(:,1));hold on;
%     subplot(3,2,2);
%     plot(a,SData_X1(:,3));hold on;
%     subplot(3,2,3);
%     plot(a,SData_X1(:,5));hold on;
%     subplot(3,2,4);
%     plot(a,SData_X1(:,7));hold on;
%     subplot(3,2,5);
%     plot(a,SData_X1(:,9));hold on;
%     subplot(3,2,6);
%     plot(a,SData_X1(:,11));hold on;
% end
% if flag_x2 
%     subplot(3,2,1);
%     plot(a,SData_X2(:,1));
%     subplot(3,2,2);
%     plot(a,SData_X2(:,3));
%     subplot(3,2,3);
%     plot(a,SData_X2(:,5));
%     subplot(3,2,4);
%     plot(a,SData_X2(:,7));
%     subplot(3,2,5);
%     plot(a,SData_X2(:,9));
%     subplot(3,2,6);
%     plot(a,SData_X2(:,11));
% end
% 
% subplot(3,2,1);
% plot(a,SData_X3(:,1));
% subplot(3,2,2);
% plot(a,SData_X3(:,3));
% subplot(3,2,3);
% plot(a,SData_X3(:,5));
% subplot(3,2,4);
% plot(a,SData_X3(:,7));
% subplot(3,2,5);
% plot(a,SData_X3(:,9));
% subplot(3,2,6);
% plot(a,SData_X3(:,11));
    
% subplot(3,2,1);hold on;
% plot(a,armjoints(:,1),'Color',[0.2117,0.2117,0.2117],'LineWidth',1.5);
% subplot(3,2,2);hold on;
% plot(a,armjoints(:,2),'Color',[0.2117,0.2117,0.2117],'LineWidth',1.5);
% subplot(3,2,3);hold on;
% plot(a,armjoints(:,3),'Color',[0.2117,0.2117,0.2117],'LineWidth',1.5);
% subplot(3,2,4);hold on;
% plot(a,armjoints(:,4),'Color',[0.2117,0.2117,0.2117],'LineWidth',1.5);
% subplot(3,2,5);hold on;
% plot(a,armjoints(:,5),'Color',[0.2117,0.2117,0.2117],'LineWidth',1.5);
% subplot(3,2,6);hold on;
% plot(a,armjoints(:,6),'Color',[0.2117,0.2117,0.2117],'LineWidth',1.5);
% 
% subplot(3,2,1);hold on;
% plot(a,PnpResult1(:,1),'Color',[0.6,0.6,0.6],'LineWidth',1.5);
% subplot(3,2,2);hold on;
% plot(a,PnpResult1(:,2),'Color',[0.6,0.6,0.6],'LineWidth',1.5);
% subplot(3,2,3);hold on;
% plot(a,PnpResult1(:,3),'Color',[0.6,0.6,0.6],'LineWidth',1.5);
% subplot(3,2,4);hold on;
% plot(a,PnpResult1(:,4),'Color',[0.6,0.6,0.6],'LineWidth',1.5);
% subplot(3,2,5);hold on;
% plot(a,PnpResult1(:,5),'Color',[0.6,0.6,0.6],'LineWidth',1.5);
% subplot(3,2,6);hold on;
% plot(a,PnpResult1(:,6),'Color',[0.6,0.6,0.6],'LineWidth',1.5);
a = 1:m;
figure;
subplot(3,2,1);
plot(a,SData_X1(:,1));hold on;
subplot(3,2,2);
plot(a,SData_X1(:,2));hold on;
subplot(3,2,3);
plot(a,SData_X1(:,3));hold on;
subplot(3,2,4);
plot(a,SData_X1(:,4));hold on;
subplot(3,2,5);
plot(a,SData_X1(:,5));hold on;
subplot(3,2,6);
plot(a,SData_X1(:,6));hold on;
subplot(3,2,1);
plot(a,SData_X2(:,1));
subplot(3,2,2);
plot(a,SData_X2(:,2));
subplot(3,2,3);
plot(a,SData_X2(:,3));
subplot(3,2,4);
plot(a,SData_X2(:,4));
subplot(3,2,5);
plot(a,SData_X2(:,5));
subplot(3,2,6);
plot(a,SData_X2(:,6));

subplot(3,2,1);
plot(a,SData_X3(:,1),'Color',[0.2117,0.2117,0.2117]);
subplot(3,2,2);
plot(a,SData_X3(:,2),'Color',[0.2117,0.2117,0.2117]);
subplot(3,2,3);
plot(a,SData_X3(:,3),'Color',[0.2117,0.2117,0.2117]);
subplot(3,2,4);
plot(a,SData_X3(:,4),'Color',[0.2117,0.2117,0.2117]);
subplot(3,2,5);
plot(a,SData_X3(:,5),'Color',[0.2117,0.2117,0.2117]);
subplot(3,2,6);
plot(a,SData_X3(:,6),'Color',[0.2117,0.2117,0.2117]);


