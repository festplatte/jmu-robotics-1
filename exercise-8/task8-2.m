global x = [11/144, -11/16, 0, 2, -11/1024, 0, -33/16, 5];

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
  global x;
  y = x(1) * t.^3 + x(2) * t.^2 + x(3) * t + x(4);
end

% second path
function y = w2(t)
  global x;
  y = x(5) * (t - 3).^3 + x(5) * (t - 3).^2 + x(7) * (t - 3) + x(8);
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
  global x;
  y = 3 * x(1) * t.^2 + 2 * x(2) * t + x(3);
end

% second path
function y = dw2(t)
  global x;
  y = 3 * x(5) * (t - 3).^2 + 2 * x(6) * (t - 3) + x(7);
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
  global x;
  y = 6 * x(1) * t + 2 * x(2);
end

% second path
function y = ddw2(t)
  global x;
  y = 6 * x(5) * (t - 3) + 2 * x(6);
end
