clear all
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%speed of light = 3e8
fc = 77e+9;
range_max = 200;
range_resolution = 1;
max_velocity = 100;
c = 3e+8;
%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
pos0 = 50; % target initial position
vel0 = 30; % target initial velocity


%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.
B = c / (2 * range_resolution); % bandwidth
Tchirp = 5.5 * 2 * range_max / c; % chirp time
k = B / Tchirp; % slope

%Operating carrier frequency of Radar 
% fc= 77e9;             %carrier freq

                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = pos0 + (vel0 * t(i)); % constant velocity motion model
    td(i) = (2 * r_t(i)) / c; % time delay to go hit the target and come back to the radar reciever.
    
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2 * pi *(fc * t(i) + 0.5 * k * t(i)^2)); % transmit wave equation.
    Rx (i) = cos(2 * pi *(fc * (t(i) - td(i)) + 0.5 * k * (t(i) - td(i))^2)); % time delayed recieved wave equation. 
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i) * Rx(i);
    
end

%% plot
plot(Tx(1:1000:end));
plot(Rx(1:1000:end));
plot(Mix(1:1000:end));

%% RANGE MEASUREMENT


 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix_Reshape = reshape(Mix, Nr, Nd);

 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
sig_fft1 = fft(Mix_Reshape, Nr);

 % *%TODO* :
% Take the absolute value of FFT output
sig_fft1 = abs(sig_fft1);
sig_fft1 = sig_fft1 ./ max(sig_fft1); % normalize

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
sig_fft1_onesided = sig_fft1(1:Nr/2);

%plotting the range
figure ('Name','Range from First FFT')
subplot(2,1,1)

 % *%TODO* :
 % plot FFT output 
plot(sig_fft1_onesided);
 
axis ([0 200 0 1]);
title('Range from First FFT');
ylabel('Normalized Amplitude');
xlabel('Range');

%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has response in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
training_cells = 10;
training_band = 8;

% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
guard_cells = 4;
guard_band = 4;

% *%TODO* :
% offset the threshold by SNR value in dB
offset = 1.4;

% *%TODO* :



% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.


% Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
% CFAR
RDM = RDM / max(RDM(:));

for row1 = training_cells + guard_cells + 1 :(Nr/2) - (training_cells + guard_cells)
    for col1 = training_band + guard_band + 1 : (Nd) - (training_band + guard_band)
        %Create a vector to store noise_level for each iteration on training cells
         noise_level = zeros(1,1);

         for row2 = row1 - (training_cells + guard_cells) : row1 + (training_cells + guard_cells)
             for col2 = col1 - (training_band + guard_band) : col1 + (training_band + guard_band)
                 if (abs(row1 - row2) > guard_cells || abs(col1 - col2) > guard_band)
                     noise_level = noise_level + db2pow(RDM(row2, col2));
                 end
             end
         end

         % Calculate threshold from noise average then add the offset
         threshold = pow2db(noise_level / (2 * (training_band + guard_band + 1) * 2 * (training_cells + guard_cells + 1) - (guard_cells * guard_band) - 1));
         threshold = threshold + offset;

         cell_under_test = RDM(row1,col1);

         if (cell_under_test < threshold)
             RDM(row1, col1) = 0;
         else
             RDM(row1, col1) = 1;
         end

    end
end
    
% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 
RDM(RDM~=0 & RDM~=1) = 0; 

% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure('Name', 'CA-CFAR Filtered RDM')
figure,surf(doppler_axis,range_axis,RDM);
colorbar;
title( 'CA-CFAR Filtered RDM surface plot');
xlabel('Speed');
ylabel('Range');
zlabel('Normalized Amplitude');
view(315, 45);

 
 