%th is a vector containing current joint positions of form [th1 th2 ... thn] 
%d_th is a vector containing joint step-sizes of form [d_th1 d_th2 ... d_thn]
%l is a vector of joint lengths of form [l1 l2... ln]
%ee is the desired end effector of form [x;y;z]
%err is a scalar for the magnitude of acceptable error from the desired end-effector
function theta = IK_Jacobian(l,th,d_th,ee,err)
	[retl,n] = size(l);
	[retth,nth] = size(th);
	[retd,nd] = size(d_th);
	[eer,eec] = size(ee);
	if((retl!=1) || (retth!=1) || (retd!=1))
		theta = 'l[], th[], and d_th[] should be vectors with one row';
	elseif((n!=nth) || (n!=nd))
		theta = 'l[], th[], and d_th[] should be vectors of the same dimension';
	elseif((eer!=3) || (eec!=1))
		theta = 'ee[] should be a vector with one column and three rows';
	else
		theta_step = th;
		X = FK(l,th);
		while(norm(ee-X)>abs(err))
			theta_step = IK_Jacob_Step(l,theta_step,d_th,ee);
			X = FK(l,theta_step);
		end
	Final_Position = X
	theta = theta_step;
	end	
end
		
function theta_new = IK_Jacob_Step(l,th,d_th,ee)
	[retl,n] = size(l);
	[retth,nth] = size(th);
	[retd,nd] = size(d_th);
	[eer,eec] = size(ee);
	if((retl!=1) || (retth!=1) || (retd!=1))
		theta_next = 'l[], th[], and d_th[] should be vectors with one row';
	elseif((n!=nth) || (n!=nd))
		theta_next = 'l[], th[], and d_th[] should be vectors of the same dimension';
	elseif((eer!=3) || (eec!=1))
		theta_next = 'ee[] should be a vector with one column and three rows';
	else
		J = Jacobian(l,th,d_th);
		Jp = pinv(J);
		X = FK(l,th);
		dX = ee-X;
		Dt = Jp*dX;
		theta_new = th+rot90(Dt);
	end
end

function J = Jacobian(l,th,d_th)
	[retl,n] = size(l);
	[retth,nth] = size(th);
	if((retl!=1) || (retth!=1))
		J = 'l[] and th[] should be vectors with one row';
	elseif(n!=nth)
		J = 'l[] and th[] should be vectors of the same dimension';
	else
		Matrix = zeros(3,n);
		for i=1:n
			d_thi = zeros(1,n);
			d_thi(i) = d_th(i);
			Matrix(1:3,i:i) = (FK(l,th+d_thi)-FK(l,th))/d_th(i);
		end
	end
	J = Matrix;
end

function end_effector = FK(l,th)
	[retl, n] = size(l);
	[retth,nth] = size(th);
	if((retl != 1) || (retth != 1))
		end_effector = 'error: l[] and th[] should be single-row vectors';
	elseif(n != nth)
		end_effector = 'error: l[] and th[] should be of the same size';
	else
		D = zeros(3,n);
		th_f = zeros(1,n+1);
		Rz = zeros(3,3,n);
		A = zeros(4,4,n);
		Rz_ext = zeros(4,4,n);
		T = zeros(4,4,n);
		T_f = 0;
		for k = 1:n
			th_f(k+1) = th_f(k) + th(k);
			D(1:3,k:k) = [l(k);0;0];
			Rz(:,:,k) = [cos(th_f(k+1)) -sin(th_f(k+1)) 0; sin(th_f(k+1)) cos(th_f(k+1)) 0; 0 0 1];
			A(:,:,k) = [Rz(:,:,k), D(1:3,k:k); 0 0 0 1];
			Rz_ext(:,:,k) = [Rz(:,:,k), [0;0;0]; 0 0 0 1];
			T(:,:,k) = Rz_ext(:,:,k) * A(:,:,k);
			T_f = T_f + T(:,:,k);
		end
		end_effector = T_f(1:3,4:4);
	end	
end
