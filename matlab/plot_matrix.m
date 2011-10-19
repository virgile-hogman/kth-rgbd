function plot_matrix( M)

    % matrix begins at M(:,2)
    % x=M(:,5) y=M(:,9) z=M(,13)
    % referential:
    % x+ is lateral right
    % y+ is upward
    % z+ is forward
    % positions seen from "above" are (z,x)
    plot(M(:,13),M(:,5),'-');
    plot(M(1,13),M(1,5),'dk');
    plot(M(end,13),M(end,5),'xk');

    text(M(1,13),M(1,5),...
         '\leftarrowstart',...
         'FontSize',10)

     text(M(end,13),M(end,5),...
         '\leftarrowend',...
         'FontSize',10)

    xlabel('z (meters)');
    ylabel('x (meters');

end

