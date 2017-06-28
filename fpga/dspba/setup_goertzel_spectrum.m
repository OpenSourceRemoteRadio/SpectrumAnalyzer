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
%% goertzel_spectrum_param - DSPBA Design Parameters Start
clear goertzel_spectrum_param; 

%% System Parameters
goertzel_spectrum_param.ClockRate               = 245.76;               % The system clock rate in MHz
goertzel_spectrum_param.ClockMargin             = 0.0;                  % Adjust the pipelining effort

%% Data Type Specification
goertzel_spectrum_param.input_word_length       = 16;                   % Input data: bit width
goertzel_spectrum_param.input_fraction_length   = 15;                   % Input data: fraction width

goertzel_spectrum_param.output_word_length      = 16;                   % Output data: bit width
goertzel_spectrum_param.output_fraction_length  = 15;                   % Output data: fraction width

%% Parameters

goertzel_flush                                  = 4;
b_coeff_reg                                     = 8;

y0_real_val                                     = 12;
y0_imag_val                                     = 16;
y1_real_val                                     = 20;
y1_imag_val                                     = 24;


bin_point_val                                   = 338;
bin_len                                         = 4096;
data_len                                        = 4096;


b_coeff                                         = zeros(bin_len,1);
W1_coeff                                        = zeros(bin_len,1);
for loop1 = 1:bin_len
        pi_term = ((2*pi*loop1)/bin_len);
        W = exp(pi_term*1i);
        W1 = exp(-1i*pi_term);
        b = 2*real(W);
        b_coeff(loop1) = b;
        W1_coeff(loop1) = W1;
end


b_coeff_val = b_coeff(bin_point_val);

coef_quant=quantizer('fixed','round','saturate',[16 13]);
b_coeff_val_quant=quantize(coef_quant,b_coeff_val)*2^13;
load_bcoeff=typecast(int16(b_coeff_val_quant),'uint16')


%% Simulation Parameters
goertzel_spectrum_param.SampleTime  = 1;                                  % One unit in Simulink simulation is one clock cycle 
goertzel_spectrum_param.endTime     = ((data_len*4))+500;                 % How many simulation clock cycles


%% Stimulus data setup

z_data=load('goertzel_data_input_tc3.txt');
a=z_data(1:2:end);
b=z_data(2:2:end);
figure(1);plot(a)
figure(2);plot(b)


goertzel_spectrum_param.inputdata1=zeros(1,4096);
goertzel_spectrum_param.inputdata2=zeros(1,4096);

goertzel_spectrum_param.inputdata1(1,:)=a;
goertzel_spectrum_param.inputdata2(1,:)=b;

%% Derived Parameters 
goertzel_spectrum_param.Period          = 4;                              % Clock cycles between consecutive data samples for a particular channel


%% DSPBA Design Parameters End
