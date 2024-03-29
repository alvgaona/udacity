clear
clc

%% RADAR SPECIFICATIONS 

carrierFrequency = 77e9;    % Frequency [Hz]
maxRange = 200;             % Max range [m]
rangeResolution = 1;        % Range resolution [m]
maxVelocity = 100;          % Max velocity [m/s]

c = 3e8;                    % Speed of light [m/s]

%% USER DEFINITIONS

% Range cannot exceed 200m and velocity can be any value between
% -70 and 70 m/s

targetRange = 100;     % Initilized range [m]
targetVelocity = -20;  % Initilized velocity [m/s]

%% FMCW WAVEFORM GENERATION

B = c / (2*rangeResolution);     % Bandwidth  [Hz]
T = 5.5 * 2 * maxRange / c;      % Chirp time [s]
slope = B/T;                     % Chirp slope [Hz/s]
disp(slope)
                                                      
Nd=128;  % Number of doppler cells or number of sent periods % number of chirps
Nr=1024; % The number of samples on each chirp. 

t=linspace(0,Nd*T,Nr*Nd);        % Total time for samples

Tx = zeros(1,length(t));         % Transmitted signal
Rx = zeros(1,length(t));         % Received signal
beatSignal = zeros(1,length(t)); % Beat signal

%% SIGNAL GENERATION AND MOVING TARGET

for i=1:length(t)                
    if(i > 1)
        targetRange = targetRange + targetVelocity * (t(i) - t(i - 1));
    end
    
    Tx(i) = cos(2*pi*(carrierFrequency*t(i) + slope * t(i)^2 / 2));
    tripTime = targetRange / c * 2;
    Rx(i) = cos(2*pi*(carrierFrequency*(t(i)-tripTime) + slope * (t(i) - tripTime)^2 / 2));
 
    beatSignal(i) = Tx(i) * Rx(i);    
end

%% RANGE MEASUREMENT

beatMat = reshape(beatSignal, [Nr, Nd]);

firstBeat = beatMat(:,Nd);
firstBeatFFT = fftshift(fft(firstBeat));

firstBeatEnergy = sumsqr(abs(firstBeatFFT))/Nr;  % Paarseval's energy
normalizedBeatFFT = abs(firstBeatFFT/firstBeatEnergy);

f = Nr/2*linspace(-1,1,Nr);

figure('Name', 'Range from first FFT')
plot(f(Nr/2:end-1),normalizedBeatFFT(Nr/2:end-1))
title('Range from first FFT')
xlabel('Range [m]','interpreter','latex')
axis ([0 200 0 1]);
grid on

%% RANGE DOPPLER RESPONSE

beatFFT2 = fft2(beatMat,Nr,Nd);

beatFFT2 = beatFFT2(1:Nr/2,1:Nd);
beatFFT2 = fftshift(beatFFT2);
RDM = abs(beatFFT2);               % Range Doppler Map (RDM)
RDM = 10*log10(RDM);               % Range Doppler Map (RDM) [dB]

figure('Name', '2D FFT')
surf(100*linspace(-1,1,Nd),200*linspace(-1,1,Nr/2)*((Nr/2)/400),RDM);
title('2D FFT')
xlabel('Doppler [${\frac{m}{s}}$]','interpreter','latex')
ylabel('Range [m]','interpreter','latex')
colorbar;

%% CFAR implementation

Tr = 10;               % Number of training cells (range dimension)
Td = 8;                % Number of training cells (doppler dimension)
Gr = 4;                % Number of guard cells (range dimension)
Gd = 4;                % Number of guard cells (doppler dimension)
offset = 10;           % Offset the threshold by SNR [dB]

numTrainingCells = 2 * (Td + Gd + 1) * 2 * (Tr + Gr + 1) - (Gr * Gd) - 1);

result = zeros(size(RDM));

for i = 1 + Tr + Gr : Nr / 2 - (Gr + Tr)
    for j = 1 + Td + Gd : Nd - (Td + Gd)
        
        noise_level = 0;
        for p = i - (Tr + Gr) : i + Tr + Gr
            for q = j - (Td + Gd) : j + Td + Gd
                if (abs(i - p) > Gr || abs(j - q) > Gd)
                    noise_level = noise_level + db2pow(RDM(p,q));                    
                end
            end
        end
        threshold = pow2db(noise_level / numTrainingCells;
        threshold = threshold + offset;
        CUT = RDM(i, j);
        
        if (CUT < threshold)
            result(i, j) = 0;
        else
            result(i, j) = 1;
        end
    end
end

figure('Name','CFAR Output')
surf(100*linspace(-1,1,Nd),200*linspace(-1,1,Nr/2)*((Nr/2)/400), result);
title('CFAR Output')
xlabel('Doppler [${\frac{m}{s}}$]','interpreter','latex')
ylabel('Range [m]','interpreter','latex')
colorbar;
