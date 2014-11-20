%l is a vector of joint lengths of form [l1 l2... ln]
%th is a vector containing current joint positions of form [th1 th2 ... thn] 
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
