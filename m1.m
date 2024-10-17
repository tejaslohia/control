d=[2 25];
n=[1 4 25];

sys_tf=tf(d,n)

%step(sys_tf)
[y t]=step(sys_tf);

stepinfo(y,t)

plot(t,y)
grid on

