Pn = sym('Pn');
Pn2 = sym('Pn2');
Pe = sym('Pe');
Pe2 = sym('Pe2');
Alt = sym('Alt');
Alt2 = sym('Alt2');

Vg = sym('Vg')
X = sym('X');
wn = sym('wn');
we = sym('we');
Y = sym('Y');
Va = sym('Va');
q = sym('q');
r = sym('r');
phi = sym('phi');
g = sym('g');
Alt_vv = sym('Alt_vv')


f=sqrt((Pn-Pn2)^2+(Pe-Pe2)^2+(Alt-Alt2)^2);
    
x = [Pn; Pe; Vg; X; wn; we; Y; Alt;Alt_vv];

for j=1:length(x)
    F(:,j) = diff(f,x(j));
end

pretty(simplify(F))