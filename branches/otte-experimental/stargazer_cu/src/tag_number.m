function number = tag_number(M)
M
s = size(M);

if(s(1) == 3 && s(2) == 3)
    cleaned = M.*[0 1 0; 1 1 1; 0 1 0]; 
elseif(s(1) == 4 && s(2) == 4)
    cleaned = M.*[0 1 1 0; 1 1 1 1; 1 1 1 1; 0 1 1 0]; 
end

hex_mat = reshape(2.^(0:15), [4, 4]);
number_mat = hex_mat.*cleaned;

if(s == 3)
    number_mat = number_mat(1:3,1:3);
end

number = sum(sum(number_mat))