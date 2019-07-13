R = 0.05;
%Base at (2,0,1)
x0=0;y0=0;z0=0;
% Height = 10
h=1;
[x,y,z]=cylinder(R, 50);
%x=x+x0;
%y=y+y0;
%z=z*h+z0;
% to plot
%surf(x,y,z);
[xh,yh,zh]=cylinder(0.065, 50);
xh0=0;yh0=0;zh0=0;
xh=xh+xh0;
yh=yh+yh0;
zh=zh*0+zh0;

figure
hm = surf(x,y,z);
hold on;
hm_h = mesh(xh,yh,zh);
%rotate(hm, [1 0 0], 45);
