d=[2 25];
n=[1 4 25];

sys_tf=tf(d,n)

[y t]=step(sys_tf);
plot(t,y)
grid on
