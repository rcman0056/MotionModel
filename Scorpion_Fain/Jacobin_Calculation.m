Vg = sym('Vg');
Pn = sym('Pn');
Pe = sym('Pe');
X = sym('X');
wn = sym('wn');
we = sym('we');
Y = sym('Y');
Va = sym('Va');
q = sym('q');
r = sym('r');
phi = sym('phi');
g = sym('g');

theta = sym('theta');

Ydot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

f=[Vg * cos(X);...
    Vg * sin(X);...
    ((Va * cos(Y)+wn)*(-Va*Ydot*sin(Y)) + (Va*sin(Y)+we)*(Va*Ydot*cos(Y)))/Vg;...
    g/Vg*tan(phi)*cos(X-Y);...
    0;...
    0;...
    Ydot];
    
x = [Pn; Pe; Vg; X; wn; we; Y];
u = [Va;q;r;phi;theta];
for j=1:length(u)
    F(:,j) = diff(f,u(j));
end

pretty(simplify(F))