 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %  
 % Copyright (c) 2017, BigCat Wireless Pvt Ltd
 % All rights reserved.
 % 
 % Redistribution and use in source and binary forms, with or without
 % modification, are permitted provided that the following conditions are met:
 % 
 %     * Redistributions of source code must retain the above copyright notice,
 %       this list of conditions and the following disclaimer.
 %
 %     * Redistributions in binary form must reproduce the above copyright
 %       notice, this list of conditions and the following disclaimer in the
 %       documentation and/or other materials provided with the distribution.
 %
 %     * Neither the name of the copyright holder nor the names of its contributors
 %       may be used to endorse or promote products derived from this software
 %       without specific prior written permission.
 % 
 % 
 % 
 % THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 % AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 % IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 % DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 % FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 % DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 % SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 % CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 % OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 % OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 % 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
fs                      = 491.52e6
bin_point_val           = 338;
bin_len                 = 4096;
data_len                = 4096;
nf                      = zeros(1,bin_len);
for z=1:bin_len
    if (z>bin_len/2) 
       nf(z)            = (z-bin_len)*(fs/bin_len);
    else
       nf(z)            = z*(fs/bin_len);
    end
end

b_coeff                 = zeros(bin_len,1);
W1_coeff                = zeros(bin_len,1);

for loop1=1:bin_len
pi_term                 = ((2*pi*loop1)/bin_len);
W                       = exp(pi_term*1i);
W1                      = exp(-1i*pi_term);
b                       = 2*real(W);
b_coeff(loop1)          = b;
W1_coeff(loop1)         = W1;
end

w1_coeff                = W1_coeff(bin_point_val)


y0_real                 = ScopeData_final.signals(1,1).values;
y1_real                 = ScopeData_final.signals(1,2).values;

y0_imag                 = ScopeData_final.signals(1,3).values;
y1_imag                 = ScopeData_final.signals(1,4).values;

y0                      = complex(y0_real(16396),y0_imag(16396))
y1                      = complex(y1_real(16396),y1_imag(16396))
data_out                = y0-w1_coeff*y1


abs(data_out)
bin_data_out            = zeros(1,bin_len);
bin_data_out(bin_point_val) = abs(data_out);

figure(1);stem(nf,bin_data_out);