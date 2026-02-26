%%Andrew Seltz Mech Des 2 HW. Shaft and Bearing Design
clc
clear
close all

F1 = 340;   %N
F2 = 520;   %N

a = linspace(.03,.12,10);
RBY = zeros(1,10);
RAY = zeros(1,10);
syms x
figure
hold on

for i = 1:10
    
    RBY(i) = -(F1.*a(i) +F2.*(a(i)+.010))./(2.*a(i) + .01);
    RAY(i) = -(F2+F1)-RBY(i);
    M = piecewise( ...
    0 <= x & x < a(i), RAY(i)*x, ...
    a(i) <= x & x < a(i)+0.01, RAY(i)*x + F1*(x - a(i)), ...
    a(i)+0.01 <= x & x <= 2*a(i)+0.01, RAY(i)*x + F1*(x - a(i)) + F2*(x - a(i)-0.01) );

    fplot(M,[0 2*a(i)+0.01])
end

xlabel('x (m)')
ylabel('Moment (N·m)')
title('Moment Diagram for Different Shaft Lengths a')
grid on
hold off

figure
plot(a, RAY);
hold on
plot(a, RBY);

title('Raction Forces at Bearing A and B')
grid on
ylabel('Reaction Force (N)')
xlabel('x (m)')

hold off




%{
Moment increases while Reaction Forces decrease so placing the bearings at
distances of around a=8cm seems to be optimal

%}
