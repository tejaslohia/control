t=0:0.2:10;

zeta=[0 0.2 0.4 0.6 0.8 1.0];
length(t)
a=[]
for n=1:6
  num = [1];
  den = [1 2*zeta(n) 1];
  sys_tf=tf(num,den)
  [y, t]=step(sys_tf,t);
  a=[a y]
end
b=a'
mesh(t,zeta,b)
