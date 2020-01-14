function [Out] = createEvalPTPTraj(t,t_init,t_final,P_init,P_final,pP_init,pP_final,ppP_init,ppP_final)

% This file was generated automatically by "symModel_UR10_3".

for i=1:length(P_init)
  p_init=P_init(i,1);
  p_final=P_final(i,1);
  pp_init=pP_init(i,1);
  pp_final=pP_final(i,1);
  ppp_init=ppP_init(i,1);
  ppp_final=ppP_final(i,1);

  a(1,1) = p_init;
  a(2,1) = pp_init;
  a(3,1) = ppp_init/2;
  a(4,1) = (20*p_final - 20*p_init - 12*pp_init*t_final - 8*pp_final*t_final + 12*pp_init*t_init + 8*pp_final*t_init - 3*ppp_init*t_final^2 + ppp_final*t_final^2 - 3*ppp_init*t_init^2 + ppp_final*t_init^2 + 6*ppp_init*t_final*t_init - 2*ppp_final*t_final*t_init)/(2*(t_final - t_init)^3);
  a(5,1) = -(30*p_final - 30*p_init - 16*pp_init*t_final - 14*pp_final*t_final + 16*pp_init*t_init + 14*pp_final*t_init - 3*ppp_init*t_final^2 + 2*ppp_final*t_final^2 - 3*ppp_init*t_init^2 + 2*ppp_final*t_init^2 + 6*ppp_init*t_final*t_init - 4*ppp_final*t_final*t_init)/(2*(t_final - t_init)^4);
  a(6,1) = (12*p_final - 12*p_init - 6*pp_init*t_final - 6*pp_final*t_final + 6*pp_init*t_init + 6*pp_final*t_init - ppp_init*t_final^2 + ppp_final*t_final^2 - ppp_init*t_init^2 + ppp_final*t_init^2 + 2*ppp_init*t_final*t_init - 2*ppp_final*t_final*t_init)/(2*(t_final - t_init)^5);

  p_d_w{i}=a(1,1)+a(2,1)*(t-t_init)+a(3,1)*(t-t_init)^2+a(4,1)*(t-t_init)^3+a(5,1)*(t-t_init)^4+a(6,1)*(t-t_init)^5;
  pp_d_w{i}=a(2,1)+2*a(3,1)*(t-t_init)+3*a(4,1)*(t-t_init)^2+4*a(5,1)*(t-t_init)^3+5*a(6,1)*(t-t_init)^4;
  ppp_d_w{i}=2*a(3,1)+6*a(4,1)*(t-t_init)+12*a(5,1)*(t-t_init)+20*a(6,1)*(t-t_init)^3;
end

Out = [p_d_w{1};p_d_w{2};p_d_w{3};pp_d_w{1};pp_d_w{2};pp_d_w{3};ppp_d_w{1};ppp_d_w{2};ppp_d_w{3}];
end 
