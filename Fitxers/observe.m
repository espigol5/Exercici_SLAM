function [y, J_y_rob, J_y_lmk] = observe(rob, lmk)

% Transform to robot frame
[lmkrob, J_lmkrob_rob, J_lmkrob_lmk]= toFrame2D(rob, lmk);

% Transform to polar coordinates
[y, J_y_lmkrob]= c2p(lmkrob);

% Chain rules for Jacobians
J_y_rob= J_y_lmkrob * J_lmkrob_rob; % Regla de la cadena, anem de y a rob

J_y_lmk= J_y_lmkrob * J_lmkrob_lmk; % Regla de la cadena, anem de y a lmk

end

function f()
%%
syms x y th px py real
r= [x;y;th];
p= [px;py];

[yy, J_yy_r, J_yy_p]= observe (r,p);
simplify(jacobian(yy,r) - J_yy_r) % Restem en els dos casos per comprovar que ha de donar 0.
simplify(jacobian(yy,p) - J_yy_p)
end