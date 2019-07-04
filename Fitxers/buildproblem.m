function [A, r]= buildproblem(states, factor)

K= numel(factor); % numel = número d'elements

A= zeros(25, 17);
r= zeros(25, 1);

row= 1;

    for k= 1:K

        y= factor{k}.measurement;
        W= (factor{k}.covariance)^-1;
        Wt2= chol(W, 'upper'); % chol és arrel quadrada d'una matriu. Wt2, es omega T/2 dels apunts.

        switch factor{k}.type
            case 'motion'
                i= factor{k}.index(1);
                j= factor{k}.index(2);
                rob1= states{1+ i}; % estat 1. 1+ és pk no doni 0 i matlab peti
                rob2= states{1+ j}; % estat 2
                [e, J_e_rob1, J_e_rob2] = error_move(rob1.value, rob2.value, y);

                rows= [row : (row +numel(y) - 1)]; %#ok<NBRAK>

                r(rows)= Wt2 * e;
                A(rows, rob1.range)= Wt2 * J_e_rob1;
                A(rows, rob2.range)= Wt2 * J_e_rob2;

            case 'lmk'
                i= factor{k}.index(1);
                j= factor{k}.index(2);
                rob= states{1+ i};
                lmk= states{1+ j};
                [e, J_e_rob, J_e_lmk] = error_observe(rob.value, lmk.value, y);

                rows= [row : (row +numel(y) - 1)]; %#ok<NBRAK>

                r(rows)= Wt2 * e;
                A(rows, rob.range)= Wt2 * J_e_rob;
                A(rows, lmk.range)= Wt2 * J_e_lmk;

            case 'pose'
                i= factor{k}.index(1);
                rob= states{1+ i};
                [e, J_e_rob]= error_pose(rob.value, y);

                rows= [row : (row +numel(y) -1)]; %#ok<NBRAK>

                r(rows)= Wt2 * e;
                A(rows, rob.range)= Wt2 * J_e_rob;


        end

        row= row + numel(y);

    end

end