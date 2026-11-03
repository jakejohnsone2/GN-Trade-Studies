function err = compute_errors(m_true, goal, fb_final, true_final, hat_final)
%COMPUTE_ERRORS Split the final miss into the budget terms (all 2x1, m).
%
%   total = e_marker + e_stop + e_nav  (exact by construction)
%     e_marker = goal - m_true          UAS: where we were told to go
%     e_stop   = fb_final - goal        D&C: where the controller thinks it stopped
%     e_nav    = true_final - fb_final  G&N: estimate error at the stop
%
%   fb_final is what the controller saw: x_hat, or the truth in perfect-
%   knowledge runs (then e_nav = 0). est is the estimator error regardless.

err.e_marker = goal - m_true;
err.e_stop   = fb_final - goal;
err.e_nav    = true_final - fb_final;
err.rover    = true_final - goal;           % 3.2.3.2: rover's own share
err.total    = true_final - m_true;         % 3.2
err.est      = true_final - hat_final;      % 3.2.2.4
end
