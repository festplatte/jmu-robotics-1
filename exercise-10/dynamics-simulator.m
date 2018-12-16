function sdd = sddot(t, T)
  a0 = 6 / T^2;
  sdd = a0 - (2*a0*t) / T;
end

function sd = sdot(t, T)
  a0 = 6 / T^2;
  sd = a0 * t - (a0*t.^2) / T;
end

function s_result = s(t, T)
  a0 = 6 / T^2;
  s_result = (a0 * t.^2) / 2 - (a0*t.^3) / (3 * T);
end

function w_result = w(t, T, w0, w1)
  w_result = w1 * s(t, T) + w0 * (1 - s(t, T));
end

function wd = wdot(t, T, w0, w1)
  wd = w1 * sdot(t, T) + w0 * (1 - sdot(t, T));
end

function wdd = wddot(t, T, w0, w1)
  wdd = w1 * sddot(t, T) + w0 * (1 - sddot(t, T));
end

function wq_result = wq(q)
  wq_result = [-sin(q(1))*q(2); cos(q(1))*q(2); 0.2; 0; 0; -1];
end

function qw_result = qw(w)
  q1 = atan2(-w(2), w(1));
  q2 = w(1) / -sin(q1);
  qw_result = [q1, q2];
end

function qd = qdot(w, wdot)
  q = qw(w);
  qd1 = (-wdot(1)*w(2) + wdot(2)*w(1)) / (w(2)^2 + w(1)^2);
  s1 = sin(q(1));
  c1 = cos(q(1));
  qd2 = (-wdot(1)*s1 + w(1)*c1*qd1) / s1^2;
  qd = [qd1, qd2];
end

function qdd = qddot(w, wdot, wddot)
  x1 = -wdot(1)*w(2) + wdot(2)*w(1);
  xd1 = -wddot(1)*w(2) + wddot(2)*w(1);
  y1 = w(2)^2 + w(1)^2;
  yd1 = 2*w(2)*wdot(2) + 2*w(1)*wdot(1);
  qdd1 = (xd1*y1 - x1*yd1) / y1^2;
  
  q = qw(w);
  qd = qdot(w, wdot);
  s1 = sin(q(1));
  c1 = cos(q(1));
  x2 = -wdot(1)*s1 + w(1)*c1*qd1;
  xd2 = -wddot(1)*s1 - wdot(1)*c1*qd(1) + wdot(1)*c1*qd(1) - w(1)*s1*qd(1)^2 + w(1)*c1*qdd1;
  y2 = s1^2;
  yd2 = 2*s1*c1*qd(1);
  qdd2 = (xd2*y2 - x2*yd2) / y2^2;
  
  qdd = [qdd1, qdd2];
end

T = 10;
t = 0:0.1:T;
sdd = sddot(t, T);
sd = sdot(t, T);
s_result = s(t, T);
plot(t,sdd);
hold on;
plot(t,sd);
hold on;
plot(t,s_result);
hold on;
