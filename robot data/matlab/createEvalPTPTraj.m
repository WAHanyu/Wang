function [Out] = createEvalPTPTraj(t,p_init,p_final,t_init,t_final)

% This file was generated automatically by "symModel_UR10_3".

a(1,1) = 0;
a(2,1) = 0;
a(3,1) = 0;
a(4,1) = 10/(t_final - t_init)^3;
a(5,1) = -15/(t_final - t_init)^4;
a(6,1) = 6/(t_final - t_init)^5;

poly = a(1,1)+a(2,1)*(t-t_init)+a(3,1)*(t-t_init)^2+a(4,1)*(t-t_init)^3+a(5,1)*(t-t_init)^4+a(6,1)*(t-t_init)^5;
ppoly = a(2,1)+2*a(3,1)*(t-t_init)+3*a(4,1)*(t-t_init)^2+4*a(5,1)*(t-t_init)^3+5*a(6,1)*(t-t_init)^4;
pppoly = 2*a(3,1)+6*a(4,1)*(t-t_init)+12*a(5,1)*(t-t_init)+20*a(6,1)*(t-t_init)^3;
p_d_w_1 = (p_final(1,1)-p_init(1,1))*poly+p_init(1,1);
p_d_w_2 = (p_final(2,1)-p_init(2,1))*poly+p_init(2,1);
p_d_w_3 = (p_final(3,1)-p_init(3,1))*poly+p_init(3,1);
pp_d_w_1 = (p_final(1,1)-p_init(1,1))*ppoly;
pp_d_w_2 = (p_final(2,1)-p_init(2,1))*ppoly;
pp_d_w_3 = (p_final(3,1)-p_init(3,1))*ppoly;
ppp_d_w_1 = (p_final(1,1)-p_init(1,1))*pppoly;
ppp_d_w_2 = (p_final(2,1)-p_init(2,1))*ppoly;
ppp_d_w_3 = (p_final(3,1)-p_init(3,1))*ppoly;

Out = [p_d_w_1;p_d_w_2;p_d_w_3;pp_d_w_1;pp_d_w_2;pp_d_w_3;ppp_d_w_1;ppp_d_w_2;ppp_d_w_3];
end 
