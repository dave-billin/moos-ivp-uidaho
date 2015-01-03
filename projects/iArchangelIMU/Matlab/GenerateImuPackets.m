function GenerateImuPackets( NumPackets, EnableTruncatedPrePacket )
% GENERATEIMUTESTDATA Generates IMU packets for testing
%  This function writes a number of IMU and AHRS packets to a file.  Sensor
%  values in these packets will consist of 1-Hz linear ramp waves between
%  the values -1.0 and 1.0.
%   
% PARAM NumPackets
%   The number of data packets to generate
%
% PARAM EnableTruncatedPrePacket
%   If set to true, all but the first data Byte of a packet will be 
%   prepended to generated data.  This allows the receiver to test its
%   ability to "re-synchronize" to incoming data.
%
    %clear all;
    close all;
    clc;
    
    fprintf('\n***  IMU packet generator    ***\n');
    fprintf('***  Written by Dave Billin  ***\n\n');
    
    %-----------------------------------
    % Prompt user for file to write to
    %-----------------------------------
    [FileName Path] = uiputfile('*.dat', ...
                      'Please select a file to write to', ...
                      strcat(pwd, '/IMU.dat') );
    if  ( isequal(FileName, 0) )
       return; 
    else
        TargetFilePath = fullfile(Path, FileName);
        fid = fopen(TargetFilePath, 'w+');
        assert( (fid ~= 0), 'Failed to open %s for writing', ...
                            TargetFilePath );
    end

    
    %-------------------------------
    % Generate IMU data
    %-------------------------------
    MaxSampleValue = 1.0;   % Absolute value of max sample value
    ImuSampleRate = 100;    % Our virtual IMU sends measurements at 100 Hz
    SampleIncrement = 2.0 / ImuSampleRate;
    s = 0;
    I = IMUMessage;
    A = AHRSMessage;
    
    % Generate 'pre-packet' data
    if (EnableTruncatedPrePacket)
        PrePacket = A.Serialize();
        % Remove the first Byte of the Pre-packet data and write to file
        PrePacket = PrePacket(2 : length(PrePacket));
        fwrite(fid, PrePacket);
    end
    
    % Generate complete packet data
    PacketCount = NumPackets;
    while (PacketCount > 0)
        % Populate measurement values in packets
        I.DeltaRollAngle = s;
        I.DeltaPitchAngle = s;
        I.DeltaYawAngle = s;
        I.DeltaVLongitudinal = s;
        I.DeltaVLateral = s;
        I.DeltaVNormal = s;
        
        A.RollAngle = s;
        A.PitchAngle = s;
        A.YawAngle = s;
        A.RollRate = s;
        A.PitchRate = s;
        A.YawRate = s;
       
        % Serialize packet data
        PacketData = [ I.Serialize(), A.Serialize() ];
        
        % Write packet data to file
        fwrite(fid, PacketData);
        
        % Increment sample value
        s = s + SampleIncrement;
        if ( s > MaxSampleValue )
            s = -1.0 * MaxSampleValue;  % Wrap sample value for ramp wave
        end
            
        % Decrement packet counter
        PacketCount = PacketCount - 1;
    end
    
    
    fprintf('%d packets were written to %s\n\n', ...
            NumPackets, TargetFilePath);
    
end

