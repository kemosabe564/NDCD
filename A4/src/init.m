%% initialization

aircraft

T_span = 0:1:Tfinal;
nx = length(x01);
length_action = size(B1,2)*Tfinal;

aircraft_1.initial_state = x01;
aircraft_2.initial_state = x02;
aircraft_3.initial_state = x03;
aircraft_4.initial_state = x04;

aircraft_1.A = A1; aircraft_2.A = A2; 
aircraft_3.A = A3; aircraft_4.A = A4;

aircraft_1.B = B1; aircraft_2.B = B2; 
aircraft_3.B = B3; aircraft_4.B = B4;

input_max = umax/Tfinal;

aircraft_1.input_max = input_max;
aircraft_2.input_max = input_max;
aircraft_3.input_max = input_max;
aircraft_4.input_max = input_max;

P1 = []; P2 = []; P3 = []; P4 = [];

Q1 = []; Q2 = []; Q3 = []; Q4 = [];

matrix_1 = [];
matrix_2 = [];
matrix_3 = [];
matrix_4 = [];

for i = 1:1:Tfinal

    temp_matrix_1 = [];
    temp_matrix_2 = [];
    temp_matrix_3 = [];
    temp_matrix_4 = [];

    for j = i:-1:1

        temp_matrix_1 = [temp_matrix_1, A1^(j-1)*B1];
        temp_matrix_2 = [temp_matrix_2, A2^(j-1)*B2];
        temp_matrix_3 = [temp_matrix_3, A3^(j-1)*B3];
        temp_matrix_4 = [temp_matrix_4, A4^(j-1)*B4];

    end
    
    temp_matrix_1 = [temp_matrix_1, zeros(size(temp_matrix_1, 1), Tfinal*size(B1, 2) - size(temp_matrix_1, 2))];
    temp_matrix_2 = [temp_matrix_2, zeros(size(temp_matrix_2, 1), Tfinal*size(B2, 2) - size(temp_matrix_2, 2))];
    temp_matrix_3 = [temp_matrix_3, zeros(size(temp_matrix_3, 1), Tfinal*size(B3, 2) - size(temp_matrix_3, 2))];
    temp_matrix_4 = [temp_matrix_4, zeros(size(temp_matrix_4, 1), Tfinal*size(B4, 2) - size(temp_matrix_4, 2))];


    if (i == Tfinal)
    
        matrix_1 = temp_matrix_1;       
        matrix_2 = temp_matrix_2;      
        matrix_3 = temp_matrix_3;      
        matrix_4 = temp_matrix_4;

        P1 = [P1; zeros(size(temp_matrix_1, 1), size(P1, 2))];
        P2 = [P2; zeros(size(temp_matrix_2, 1), size(P2, 2))];
        P3 = [P3; zeros(size(temp_matrix_3, 1), size(P3, 2))];
        P4 = [P4; zeros(size(temp_matrix_4, 1), size(P4, 2))];

        Q1 = [Q1; zeros(size(A1))];
        Q2 = [Q2; zeros(size(A2))]; 
        Q3 = [Q3; zeros(size(A3))];
        Q4 = [Q4; zeros(size(A4))];

        break;

    end

    P1 = [P1; temp_matrix_1];
    P2 = [P2; temp_matrix_2];
    P3 = [P3; temp_matrix_3];
    P4 = [P4; temp_matrix_4];

    Q1 = [Q1; A1^i];
    Q2 = [Q2; A2^i];
    Q3 = [Q3; A3^i];
    Q4 = [Q4; A4^i];


end

aircraft_1.nx_input = size(P1, 2);
aircraft_2.nx_input = size(P2, 2);
aircraft_3.nx_input = size(P3, 2);
aircraft_4.nx_input = size(P4, 2);


aircraft_1.P = P1;
aircraft_2.P = P2;
aircraft_3.P = P3;
aircraft_4.P = P4;

aircraft_1.Q = Q1;
aircraft_2.Q = Q2;
aircraft_3.Q = Q3;
aircraft_4.Q = Q4;

aircraft_1.matrix = matrix_1;
aircraft_2.matrix = matrix_2;
aircraft_3.matrix = matrix_3;
aircraft_4.matrix = matrix_4;

clear i j
clear input_max
clear temp_matrix_1 temp_matrix_2 temp_matrix_3 temp_matrix_4
clear matrix_1 matrix_2 matrix_3 matrix_4
clear P1 P2 P3 P4 Q1 Q2 Q3 Q4 