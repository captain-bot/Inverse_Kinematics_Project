clc
clear
close all

% Timing test for matrix multiplication
A = [1, 2, 3, 4; 5, 6, 7, 8; 9, 10, 11, 12; 13, 14, 15, 16];
B = [1, 2, 3, 4; 5, 6, 7, 8; 9, 10, 11, 12; 13, 14, 15, 16];

start_t = cputime;
for i = 1:1000000
    A * B;
end
complete_t = cputime;
elapsed_t = complete_t - start_t;

fprintf("Time elapsed: %2.10f sec\n", elapsed_t);