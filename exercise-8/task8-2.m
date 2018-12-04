function plotTrajectory()
  t = 0:0.1:3;
  w = w1(t);
  plot(t,w);
  hold on;
  
  t = 3:0.1:8;
  w = w2(t);
  plot(t,w);
  hold on;
end

% first path
function y = w1(t)
  x = [11/16, -17/48, 0, 2];
  y = x(1) * t.^3 + x(2) * t.^2 + x(3) * t + x(4);
end

% second path
function y = w2(t)
  x = [11/1024, 0, -33/16, 5];
  y = x(1) * (t - 3).^3 + x(2) * (t - 3).^2 + x(3) * (t - 3) + x(4);
end

function plotVelocity()
  t = 0:0.1:3;
  dw = dw1(t);
  plot(t,dw);
  hold on;
  
  t = 3:0.1:8;
  dw = dw2(t);
  plot(t,dw);
  hold on;
end

% first path
function y = dw1(t)
  x = [11/16, -17/48, 0, 2];
  y = 3 * x(1) * t.^2 + 2 * x(2) * t + x(3);
end

% second path
function y = dw2(t)
  x = [11/1024, 0, -33/16, 5];
  y = 3 * x(1) * (t - 3).^2 + 2 * x(2) * (t - 3) + x(3);
end

function plotAcceleration()
  t = 0:0.1:3;
  ddw = ddw1(t);
  plot(t,ddw);
  hold on;
  
  t = 3:0.1:8;
  ddw = ddw2(t);
  plot(t,ddw);
  hold on;
end

% first path
function y = ddw1(t)
  x = [11/16, -17/48, 0, 2];
  y = 6 * x(1) * t + 2 * x(2);
end

% second path
function y = ddw2(t)
  x = [11/1024, 0, -33/16, 5];
  y = 6 * x(1) * (t - 3) + 2 * x(2);
end
