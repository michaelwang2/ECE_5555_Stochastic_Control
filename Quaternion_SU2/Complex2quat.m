function q = Complex2quat(C)
% q = [v; q4] = [b c d a]
q = [imag(C(1,1)); real(C(1,2)); imag(C(1,2)); real(C(1,1))];
end