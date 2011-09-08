% otte, 2010, this dispalys what a stargazer nXn tag should look like based
% on its number

function tag_view(num,n)

actual_num = num + 1 + 2^(n-1) + 2^(n-1) * 16^(n-1);          % adjust for corner tags
raw = dec2bin(actual_num);                                    % turn to raw binary string
formatted = fliplr([repmat('0',[1,(16)-length(raw)]) , raw]); % flip and pad
square_format = reshape(str2num(formatted(:)), [4,4]);        % make square format
square_format = square_format(1:n, 1:n)                       %reduce if necessary
 
%plot
square_format(square_format==0) = nan;
ycoords = repmat((1+n - (1:n))', [1,n]).*square_format;
xcoords = repmat(1:n, [n,1]).*square_format;



figure(1)
plot(xcoords(:), ycoords(:), 'o',      ...
                'LineWidth',2,         ...
                'MarkerEdgeColor','k', ...
                'MarkerFaceColor','r', ...
                'MarkerSize',20)
axis([0 n+1 0 n+1])