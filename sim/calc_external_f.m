function f_ext = calc_external_f(M, C, tau, Jc, qdd)

  f_ext = pinv(Jc')*(M*qdd + C' - tau)
  
end