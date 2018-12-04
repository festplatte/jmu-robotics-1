function rhino(i)
    % i: Determine which destination position to use

    % tool-configuration start position
    w0 = [ 0.45; 0; 0.05; 0; 0; -0.60653; ];

    if i == 1 % destination position 1
        w1 = [ 0.0; 0.45; 0.05; 0; 0; -0.60653; ];
    else % destination position 2
        w1 = [ 0.35; 0.40;  0.4; 0; 0; -0.60653;  ];
    end
    
    % overall time in seconds for straight-line motion
    T = 7.0;
    % duration of ramp in seconds of speed profile
    tau = 2.0;
    % sampling time in seconds
    dt = 0.1;

    % initialize current tool-configuration vector with start position
    w = w0;
    % initialize current joint variables according to inverse kinematics of
    % start position
    q = qw(w);

    % loop until time T in steps of sampling time dt
    for t = 0:dt:T
        % get value of speed distribution function (between 0 and 1)
        s = speed_dist(t, T, tau);

        % calculate desired tool-configuration vector by interpolating the
        % straight line between w0 and w1 according to s
        w_d = w0*(1 - s) + w1*s;

        % check if w_d is valid, if not qw issues a warning
        qw(w_d);

        % compute desired tool-configuration velocities to reach w_d within
        % one sampling instant
        dw_d = (w_d - w) / dt;
        
        % compute desired joint-space velocities for reaching w_d
        dq_d = dqw(w, dw_d, q);

        % simulate the real robot by applying the necessary joint-space
        % velcoties (assumption: joint velocity is constant during sampling
        % time)
        q = q + dq_d * dt;

        % get the tool-configuration vector from direct kinematics
        w = wq(q);

        % output
        disp(['t:    ' num2str(t) ', s: ' num2str(s)]);
        disp(['w_d:  ' num2str(w_d', '%12.4f')]);
        disp(['dw_d: ' num2str(dw_d', '%12.4f')]);
        disp(['dq_d: ' num2str(dq_d', '%12.4f')]);
        disp(['q:    ' num2str(q', '%12.4f')]);
        disp(['w:    ' num2str(w', '%12.4f')]);
        disp(['w_d-w: ' num2str((w_d-w)', '%12.4f')]);
        disp('-------------------------------------------------------------------------');
    end

end

% direct kinematics (tool-configuration vector)
function w = wq(q)
    T50 = Tlink(q(1), 0.2604, 0, -pi/2) * ...
          Tlink(q(2), 0, 0.2286, 0) * ...
          Tlink(q(3), 0, 0.2286, 0) * ...
          Tlink(q(4), 0, 0.0095, -pi/2) * ...
          Tlink(q(5), 0.1683, 0, 0);

    w = [T50(1:3,4); T50(1:3,3)*exp(q(5)/pi)];
end

% inverse kinematics
function q = qw(w)
    d1 = 0.2604;
    a2 = 0.2286;
    a3 = 0.2286;
    a4 = 0.0095;
    d5 = 0.1683;
    
    q1 = atan2(w(2), w(1));
    q234 = atan2(-(cos(q1)*w(4) + sin(q1)*w(5)), -w(6));
    b1 = cos(q1)*w(1) + sin(q1)*w(2) - a4*cos(q234) + d5*sin(q234);
    b2 = d1 - a4*sin(q234) - d5*cos(q234) - w(3);
    x = (b1^2 + b2^2 - a2^2 - a3^2) / (2*a2*a3);
    if abs(x) > 1
        warning('w is invalid');
        return;
    end
    q3 = acos(x);
    
    q2 = atan2((a2+a3*cos(q3))*b2 - a3*sin(q3)*b1, (a2+a3*cos(q3))*b1 + a3*sin(q3)*b2);
    q4 = q234 - q2 - q3;
    q5 = pi*log(sqrt(w(4)^2 + w(5)^2 + w(6)^2));
    
    q = [q1; q2; q3; q4; q5];
end

% compute joint-space velocities from tool-configuration velocities
function dq = dqw(w, dw, q)
    d1 = 0.2604;
    a2 = 0.2286;
    a3 = 0.2286;
    a4 = 0.0095;
    d5 = 0.1683;

    b0 = cos(q(1))*w(4) + sin(q(1))*w(5);
    q234 = atan2(-b0, -w(6));

    b1 = cos(q(1))*w(1) + sin(q(1))*w(2) - a4*cos(q234) + d5*sin(q234);
    b2 = d1 - a4*sin(q234) - d5*cos(q234) - w(3);
    
    dq1 = (w(1)*dw(2) - w(2)*dw(1)) / (w(1)^2 + w(2)^2);
    
    db0 = cos(q(1))*dw(4) + sin(q(1))*dw(5) + (cos(q(1))*w(5) - sin(q(1)*w(4)))*dq1;
    
    dq234 = (db0*w(6) - b0*dw(6)) / (b0^2 + w(6)^2);
    
    db1 = cos(q(1))*dw(1) + sin(q(1))*dw(2) + (cos(q(1))*w(2) -sin(q(1))*w(1))*dq1 + (a4*sin(q234)+d5*cos(q234))*dq234;
    db2 = (d5*sin(q234)-a4*cos(q234))*dq234 - dw(3);
    dq3 = - 2*(b1*db1+b2*db2) / sqrt((2*a2*a3)^2 - (b1^2+b2^2-a2^2-a3^2)^2 );

    b3 = (a2+a3*cos(q(3)))*b1 + a3*sin(q(3))*b2;
    b4 = (a2+a3*cos(q(3)))*b2 - a3*sin(q(3))*b1;

    db3 = (a2+a3*cos(q(3)))*db1 + a3*sin(q(3))*db2 + a3*(cos(q(3))*b2 - sin(q(3))*b1)*dq3;
    db4 = (a2+a3*cos(q(3)))*db2 - a3*sin(q(3))*db1 - a3*(cos(q(3))*b1 + sin(q(3))*b2)*dq3;
    
    dq2 = (b3*db4 - b4*db3) / (b3^2+b4^2);
    
    dq4 = dq234 - dq2 - dq3;

    dq5 = pi * (w(4)*dw(4)+w(5)*dw(5)+w(6)*dw(6)) / (w(4)^2+w(5)^2+w(6)^2);
    
    dq = [dq1; dq2; dq3; dq4; dq5];
    
end

% compute value of speed distribution function using ramp profile
function s = speed_dist(t, T, tau)
	if (tau <= 0.0) || (tau >= T)
		if t > T
			s = 1;
		else
			s = t/T;
        end
    else
		if t < tau
			s = t^2 / (2*tau*(T-tau));
        elseif (t >= tau) && (t < T - tau)
			s = tau / (2*(T-tau)) + (t - tau) / (T - tau);
        elseif (t >= T - tau) && (t <= T)
			s = tau / (2*(T -tau)) + ...
				   (T-tau-tau) / (T-tau) + ...
				   (t - T + tau) / (T - tau) - ...
				   (t - T + tau)^2 / (2*tau*(T - tau));
		else
			s = 1;
        end
    end
end

% compute T_{k-1}^k
function T = Tlink(theta, d, a, alpha)
    T = [ ...
        cos(theta)  -cos(alpha)*sin(theta)   sin(alpha)*sin(theta) a*cos(theta) ; ...
        sin(theta)   cos(alpha)*cos(theta)  -sin(alpha)*cos(theta) a*sin(theta) ; ...
        0                   sin(alpha)               cos(alpha)         d       ; ...
        0                       0                        0              1         ...
    ];
end
