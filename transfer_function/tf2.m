s = tf('s');
sys = 1/(s^2+0.6*s+1);

t = 0:0.001:30;
[y, t] = step(sys, t);
plot(t, y);
stepinfo(y, t)

