clc;
clear all;
close all;
syms th(t) x(t) y(t) J m r z1_d(t) z2_d(t)
t = sym('t','real');

assume(x(t),'real');
assume(y(t),'real');
assume(z1_d(t),'real');
assume(z2_d(t),'real');

z1 = x(t) - (J/m*r)*sin(th(t));
z2 = y(t) + (J/m*r)*cos(th(t));

z1_dot = diff(z1,t);
z2_dot = diff(z2,t);

z1_d_dot = diff(z1_dot,t);
z2_d_dot = diff(z2_dot,t);

