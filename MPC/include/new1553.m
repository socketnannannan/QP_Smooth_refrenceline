function a = test4(u1,u2,u3,u4,u5,u6)
    %coder.extrinsic('qpOASES') 
    coder.extrinsic('quadprog') 
    MPCParameters.Np      = 25;% predictive horizon               预测步长
    MPCParameters.Nc      = 10;% control horizon                 控制步长
    MPCParameters.Nx      = 5; %number of state variables        状态变量个数
    MPCParameters.Nu      = 1; %number of control inputs         控制输入个数（a）
    MPCParameters.Ny      = 4; %number of output variables  
    MPCParameters.Ts      = 0.05; %Set the sample time           采样周期     
    MPCParameters.Q       = diag([1,1,1,1]); % cost weight factor   Q代价
    MPCParameters.R       = 1; % cost weight factor                  R代价
    MPCParameters.S       = 1; % cost weight factor                  S代价

    MPCParameters.amin      = -3.0;  % the min of deceleration   //a，j约束
    MPCParameters.amax      = 3;  % the max of acceleration
    MPCParameters.jmin     = -5.0; % minimum limits of jerk
    MPCParameters.jmax     = 5.0; % maximum limits of jerk

    MPCParameters.t   =0.1;                                     //

    Ts = MPCParameters.Ts; % 50ms
    t =  MPCParameters.t;
    Nc = MPCParameters.Nc;
    Timegap =1.4;     //车间时距
    
    StateSpaceModel.A =  [1   0    Ts  1/2*Ts*Ts  0;
                          0   1    0     Ts       0;
                          0   0    1     -Ts      0;
                          0   0    0     1-Ts/t   1;
                          0   0    0     -1/t     0];
    StateSpaceModel.B =  [0;      0;    0; Ts/t ;               1/t]; 

    StateSpaceModel.C =  [1    -Timegap   0      0      0;
                          0    0          1      0      0;
                          0    0          0      1      0;
                          0    0          0      0      1];
    
    StateSpaceModel.Z  =[10;      0;      0;      0] ;
    StateSpaceModel.L  = [  1 0 0 0 0;
                            0 1 0 0 0
                            0 0 0 1 0;
                            0 0 0 0 1];
 
    StateSpaceModel.phi   =diag([1,0,0,1]);%引入指数衰减函数 y_ref(k+i)=phi^i * y(k)
    %***********Step (1). Update vehicle states *************************% 
    RelativeDistance = u1;     %两车实际车间距
    EgoV             = u2;  %车辆纵向速度，单位：km/h-->m/s
    RelativeV        = u3;  %车辆纵向速度，单位：km/h-->m/s
    EgoAcc           = u4;  %车辆纵向加速度，单位：g's-->m/s2 
    EgoJerk          = u5; %更新车辆状态向量
    %ErrRelativeDistance = u(6);
    EgoVSet             =u6;
    dc                  =  6;  %delatx的最小值
    xk = [RelativeDistance,EgoV,RelativeV,EgoAcc,EgoJerk]';
    %****Step(3): update longitudinal vehilce model with inertial delay***%

    jmax = MPCParameters.jmax;
    jmin = MPCParameters.jmin;
    amin = MPCParameters.amin;  
    amax = MPCParameters.amax; 
    vmin =0; 
    vmax = EgoVSet;
    M = [dc;vmin;amin;jmin];
    N = [inf;vmax;amax;jmax];
    [A__,B__,C__,D__,Z__,Q__,R__,PHI__,L__] = func_Update(StateSpaceModel, MPCParameters);
    C=StateSpaceModel.C ;
    [K1,K2] = func_Update_H_f(C__,D__,Q__,R__,Z__,PHI__,xk,C);
    [M__,N__,Umax_,Umin_] =func_Constraints_(MPCParameters,M,N);
    [g,T] = func_update_Constraints_(B__,A__,M__,N__,L__,xk,Umax_,Umin_);
    U=zeros(Nc,1);
    %U = qpOASES(K1,K2,g,T); 
    % U = quadprog(K1,K2,g,T); 
   U = quadprog(K1,K2);
    a=U(1);
end


    
function [A__,B__,C__,D__,Z__,Q__,R__,PHI__,L__] = func_Update(StateSpaceModel, MPCParameters)
    coder.extrinsic('cell2mat')    
    Np = MPCParameters.Np;
    Nc = MPCParameters.Nc;
    Nx = MPCParameters.Nx;
    Ny = MPCParameters.Ny;
    Nu = MPCParameters.Nu;
    Q=MPCParameters.Q ;
    R=MPCParameters.R ;
    A = StateSpaceModel.A;
    B = StateSpaceModel.B;
    C = StateSpaceModel.C;
    
    Z = StateSpaceModel.Z;
    L = StateSpaceModel.L;

    phi = StateSpaceModel.phi; 
    A_ = cell(Np,1);        %  A_ = [A;A^2;A^3;A^4;A^5];
    B_ = cell (Np,Nc);
   
    
    C_  = cell(Np,1);
    D_  = cell(Np,Nc);
 
    Z_  = cell(Np,1);
    Q_  = cell(Np,Np);
    P_  = cell(Nc,Nc);
    %R_  = cell(Nc,Nc)
    PHI_  = cell(Np,Np);
    L_ = cell(Np,Np);
    
    
 %A_矩阵   
    for j=1:1:Np
       A_{j,1}=A^j;                    
    end
    
 %B_矩阵
 for j=1:1:Np                  
        for k=1:1:Nc
            if k<=j
                B_{j,k}=A^(j-k)*B;        %  demision:Ny*Nu
            else 
                B_{j,k}=zeros(Nx,Nu);
            end
        end
 end  
 
 

    
 %C_矩阵 
    for j=1:1:Np
       C_{j,1}=C*A^j;                    
    end   
    
  %D_矩阵 
   for j=1:1:Np
        for k=1:1:Nc
            if k<=j
                D_{j,k}=C*A^(j-k)*B;        %  demision:Ny*Nu
            else 
                D_{j,k}=zeros(4,1);
            end
        end
   end
 
 
   %Z_矩阵
    for j=1:1:Np
       Z_{j,1}=Z;                    
    end  
    
    %Q_矩阵 
    for j=1:1:Np

        for i=1:1:Np
            if i ==j 
                Q_{j,i}=Q;   
            else
               Q_{j,i} =zeros(4,4);
            end
        end
    end
   
    R_ = zeros(Nc,Nc);
     %R_矩阵 
    for j=1:1:Nc
       for i=1:1:Nc 
           if i ==j 
                R_(j,i)=1; 
           else
               R_(j,i) =0;
           end
           end 
    end
    
    %phi_矩阵
    for j=1:1:Np
        for i=1:1:Np
       if i ==j 
            PHI_{j,i}=phi^j;    
       else
          PHI_{j,i}=zeros(4,4);
       end 
        end 
    end

    %L_矩阵
      for j=1:1:Np
        for i=1:1:Np
            if i ==j 
                L_{j,i}=L;   
            else
               L_{j,i} =zeros(4,5);
            end
        end
    end
    A__=zeros(5*Np,5);
    B__=zeros(5*Np,Nc);
    C__=zeros(4*Np,5);
    D__=zeros(4*Np,Nc);
    G__=zeros(5*Np,Np);
    Z__=zeros(4*Np,1);
    Q__=zeros(4*Np,4*Np);
    %R__=zeros(Nc,Nc);
    PHI__=zeros(4*Np,4*Np);
    L__ =zeros(4*Np,5*Np);
    

    A__=cell2mat(A_); 
    B__=cell2mat(B_) ;  
    C__=cell2mat(C_) ;
    D__=cell2mat(D_) ;
    Z__=cell2mat(Z_);
    Q__=cell2mat(Q_) ;         
    %R__=cell2mat(R_);
    
    PHI__=cell2mat(PHI_);
    L__=cell2mat(L_);
    R__=R_;
end


function[K1,K2] = func_Update_H_f(C__,D__,Q__,R__,Z__,PHI__,xk,C)
    K1 = R__+D__'*Q__*D__;
    K2 = xk'*(C__'-C__'*PHI__')*Q__*D__-(Z__'-Z__'*PHI__')*Q__*D__;   %+W(k+p)'*E__'*Q__*D__+e*x(k)'*F__'*Q__*D__
    K2 = K2';
end    
  
function[M__,N__,Umax_,Umin_] =func_Constraints_(MPCParameters,M,N)
    coder.extrinsic('cell2mat')    
    
    Np= MPCParameters.Np;
    Nc= MPCParameters.Nc;
    umin =  MPCParameters.amin;
    umax =  MPCParameters.amax;

    Umin  = cell(Nc,1);
    Umax  = cell(Nc,1);
%     Umin  = cell(25,1);
%     Umax  = cell(25,1);
    M_  = cell(Np,1);
    N_ = cell(Np,1);
    %Umin矩阵
    for j=1:1:Nc
       Umin{j,1}=umin;                    
    end
 %Umax矩阵
    for j=1:1:Nc
       Umax{j,1}=umax;                    
    end
   %M_矩阵
    for j=1:1:Np
      M_{j,1}=M;                    
    end
 %N_矩阵
    for j=1:1:Np
       N_{j,1}=N;                    
    end 
%     Umax_=zeros(4*Nc,1);
%     Umin_ =zeros(4*Nc,1);
    Umax_=zeros(Nc,1);
    Umin_ =zeros(Nc,1);

    Umax_=cell2mat(Umax);           
    Umin_=cell2mat(Umin);


    M__=zeros(4*Np,1);
    N__ =zeros(4*Np,1);
    M__=cell2mat(M_);           
    N__=cell2mat(N_);
end

function  [g,T] = func_update_Constraints_(B__,A__,M__,N__,L__,xk,Umax_,Umin_)



    g  =  [L__*B__;
           -L__*B__;
             eye(10,10);
             -eye(10,10)];
    T  =  [N__-L__*A__*xk;        %-L__*G__*W(k+p)-L__*H__*ex(k)
           -M__+L__*A__*xk;       %+L__*G__*W(k+p)+L__*H__*ex(k)
            Umax_;
            Umin_];
end
