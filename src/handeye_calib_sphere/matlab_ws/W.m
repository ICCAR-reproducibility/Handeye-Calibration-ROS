function [ W ] = W(r)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
W = [r(1,1), -r(2,1), -r(3,1), -r(4,1);
        r(2,1), r(1,1), r(4,1), -r(3,1);
        r(3,1), -r(4,1), r(1,1), r(2,1);
        r(4,1), r(3,1), -r(2,1), r(1,1)];

end

